import json
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

# Niceville, FL
_START_LAT = 30.5138
_START_LON = -86.4869

_MPH_TO_MPS = 0.44704
_METERS_PER_DEG_LAT = 111_111.0
_MAX_TURN_RATE = 3.0  # degrees per second
_EARTH_RADIUS_MILES = 3958.8
_METERS_TO_MILES = 1.0 / 1609.344
_START_FUEL_MILES = 1.0
_REFUEL_DURATION_SECONDS = 20.0


def _haversine_miles(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return _EARTH_RADIUS_MILES * 2 * math.asin(math.sqrt(a))


class DroneSimNode(Node):
    def __init__(self):
        super().__init__('drone_sim')

        # Declares a value agent_id with a default value of 'drone_001'
        self.declare_parameter('agent_id', 'drone_001') 
        self._agent_id: str = self.get_parameter('agent_id').get_parameter_value().string_value

        self.declare_parameter('sim_speed', 1.0)
        self._sim_speed: float = max(0.1, self.get_parameter('sim_speed').get_parameter_value().double_value)

        self.lat = _START_LAT
        self.lon = _START_LON
        self.heading = 90.0   # degrees, 0=North, clockwise
        self.velocity = 15.0  # mph
        self._desired_heading: float | None = None
        self._fuel_range: float = _START_FUEL_MILES
        self._homeplate_lat: float = _START_LAT
        self._homeplate_lon: float = _START_LON
        self._state: str = 'AWAITING_TASKING'
        self._refuel_started: float | None = None

        ns = self._agent_id
        self._gps_pub   = self.create_publisher(NavSatFix, f'{ns}/drone/gps', 10)
        self._hdg_pub   = self.create_publisher(Float32,   f'{ns}/drone/heading', 10)
        self._vel_pub   = self.create_publisher(Float32,   f'{ns}/drone/velocity', 10)
        self._fuel_pub  = self.create_publisher(Float32,   f'{ns}/drone/fuel_range', 10)
        self._bingo_pub = self.create_publisher(Float32,   f'{ns}/drone/bingo', 10)

        self.create_subscription(Float32, f'{ns}/desired_heading', self._on_desired_heading, 10)
        self.create_subscription(Float32, f'{ns}/desired_velocity', self._on_desired_velocity, 10)
        self.create_subscription(String,  'swarm/homeplate',        self._on_homeplate, 10)
        self.create_subscription(String,  f'{ns}/state',            self._on_state, 10)

        self.create_timer(1.0, self._update) # Runs update every second
        self.get_logger().info(f'DroneSimNode started — agent_id={self._agent_id} sim_speed={self._sim_speed}x')

#----------------------------------------------------------------------------------------------------------
    def _on_desired_heading(self, msg: Float32) -> None:
        self._desired_heading = float(msg.data)

    def _on_desired_velocity(self, msg: Float32) -> None:
        self.velocity = max(0.0, min(20.0, float(msg.data)))

    def _on_homeplate(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._homeplate_lat = float(data['lat'])
            self._homeplate_lon = float(data['lon'])
        except (json.JSONDecodeError, KeyError):
            pass

    def _on_state(self, msg: String) -> None:
        new_state = msg.data
        if self._state == 'REFUELING' and new_state == 'AWAITING_TASKING':
            self._fuel_range = _START_FUEL_MILES
            self.get_logger().info(f'[{self._agent_id}] Refueling complete — fuel restored to default range ({self._fuel_range:.2f} mi)')
        self._state = new_state

    def _apply_turn_rate(self) -> None:
        if self._desired_heading is None:
            return
        # shortest angular path, clamped to max turn rate scaled by sim_speed
        diff = (self._desired_heading - self.heading + 180) % 360 - 180
        step = max(-_MAX_TURN_RATE * self._sim_speed, min(_MAX_TURN_RATE * self._sim_speed, diff))
        self.heading = (self.heading + step) % 360

    def _update(self):
        if self._state == 'REFUELING':
            if self._refuel_started is None:
                self._refuel_started = time.monotonic()
            if time.monotonic() - self._refuel_started >= _REFUEL_DURATION_SECONDS:
                self._fuel_range = _START_FUEL_MILES
                self._refuel_started = None
                self.get_logger().info(f'[{self._agent_id}] Refueling complete — fuel restored to default range ({self._fuel_range:.2f} mi)')
            else:
                self.get_logger().info(f'[{self._agent_id}] Refueling in progress — fuel={self._fuel_range:.2f} mi')
            fuel = Float32()
            fuel.data = float(self._fuel_range)
            self._fuel_pub.publish(fuel)
            return

        self._apply_turn_rate()

        # advance position scaled by sim_speed per tick
        dist_m = self.velocity * _MPH_TO_MPS * self._sim_speed
        heading_rad = math.radians(self.heading)

        self.lat += (dist_m * math.cos(heading_rad)) / _METERS_PER_DEG_LAT
        self.lon += (dist_m * math.sin(heading_rad)) / (
            _METERS_PER_DEG_LAT * math.cos(math.radians(self.lat))
        )

        self._fuel_range = max(0.0, self._fuel_range - dist_m * _METERS_TO_MILES)
        bingo = _haversine_miles(self.lat, self.lon, self._homeplate_lat, self._homeplate_lon)

        # clamp heading to [0, 360)
        self.heading = self.heading % 360.0
        # clamp velocity to [0, 20]
        self.velocity = max(0.0, min(20.0, self.velocity))

        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = self._agent_id
        gps.latitude = self.lat
        gps.longitude = self.lon
        gps.altitude = 0.0
        self._gps_pub.publish(gps)

        hdg = Float32()
        hdg.data = float(self.heading)
        self._hdg_pub.publish(hdg)

        vel = Float32()
        vel.data = float(self.velocity)
        self._vel_pub.publish(vel)

        fuel = Float32()
        fuel.data = float(self._fuel_range)
        self._fuel_pub.publish(fuel)

        bingo_msg = Float32()
        bingo_msg.data = float(bingo)
        self._bingo_pub.publish(bingo_msg)

        self.get_logger().info(
            f'lat={self.lat:.6f}  lon={self.lon:.6f}  '
            f'hdg={self.heading:.1f}°  vel={self.velocity:.1f} mph  '
            f'fuel={self._fuel_range:.2f} mi  bingo={bingo:.2f} mi'
        )

#----------------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DroneSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
