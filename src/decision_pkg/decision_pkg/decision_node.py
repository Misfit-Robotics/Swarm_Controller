import json
import math
import time
from typing import Any
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

AWAITING_TASKING = 'AWAITING_TASKING'
ENROUTE_TO_TASKING = 'ENROUTE_TO_TASKING'
SEARCHING = 'SEARCHING'
RTB = 'RTB'
REFUELING = 'REFUELING'
_DEFAULT_VELOCITY_MPH = 15.0
_HOMEPLATE_RADIUS_MILES = 0.1
_REFUEL_DURATION_SECONDS = 60.0

def _centroid(coords: list) -> tuple[float, float]:
    """Return (lat, lon) centroid of [[lon, lat], ...] polygon vertices."""
    lons = [c[0] for c in coords]
    lats = [c[1] for c in coords]
    return sum(lats) / len(lats), sum(lons) / len(lons)


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return bearing in degrees (0=N, clockwise) from point 1 to point 2."""
    dlat = lat2 - lat1
    dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    heading = math.degrees(math.atan2(dlon, dlat))
    return (heading + 360.0) % 360.0


def _point_in_polygon(lon: float, lat: float, coords: list) -> bool:
    """Return whether a [lon, lat] point is inside a polygon."""
    inside = False
    previous = coords[-1]
    for current in coords:
        current_lon, current_lat = current[0], current[1]
        previous_lon, previous_lat = previous[0], previous[1]
        crosses_latitude = (current_lat > lat) != (previous_lat > lat)
        if crosses_latitude:
            crossing_lon = ((previous_lon - current_lon) * (lat - current_lat)
                            / (previous_lat - current_lat) + current_lon)
            if lon < crossing_lon:
                inside = not inside
        previous = current
    return inside


def _haversine_miles(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    earth_radius_miles = 3958.8
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return earth_radius_miles * 2 * math.asin(math.sqrt(a))


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('agent_id', 'drone_001')
        self._agent_id: str = self.get_parameter('agent_id').get_parameter_value().string_value

        self._tasks: dict[str, dict[str, Any]] = {}
        self._lat: float | None = None
        self._lon: float | None = None
        self._fuel_range: float = float('inf')
        self._bingo: float = 0.0
        self._homeplate_lat: float = 30.5138
        self._homeplate_lon: float = -86.4869
        self._state = AWAITING_TASKING
        self._active_task: dict[str, Any] | None = None
        self._refuel_started: float | None = None

        ns = self._agent_id
        self._hdg_pub = self.create_publisher(Float32, f'{ns}/desired_heading', 10)
        self._vel_pub = self.create_publisher(Float32, f'{ns}/desired_velocity', 10)
        self._state_pub = self.create_publisher(String, f'{ns}/state', 10)

        self.create_subscription(String,    f'{ns}/tasks',           self._on_tasks,    10)
        self.create_subscription(NavSatFix, f'{ns}/drone/gps',       self._on_gps,      10)
        self.create_subscription(Float32,   f'{ns}/drone/fuel_range', self._on_fuel,    10)
        self.create_subscription(Float32,   f'{ns}/drone/bingo',     self._on_bingo,    10)
        self.create_subscription(String,    'swarm/homeplate',       self._on_homeplate, 10)
        self.create_timer(1.0, self._state_decision)
        self._publish_state("AWAITING_TASKING")

        self.get_logger().info(
            f'DecisionNode started — agent_id={self._agent_id}'
        )

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_gps(self, msg: NavSatFix) -> None:
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._state_decision()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_fuel(self, msg: Float32) -> None:
        self._fuel_range = float(msg.data)
        self._state_decision()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_bingo(self, msg: Float32) -> None:
        self._bingo = float(msg.data)
        self._state_decision()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_homeplate(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._homeplate_lat = float(data['lat'])
            self._homeplate_lon = float(data['lon'])
        except (json.JSONDecodeError, KeyError):
            pass

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_tasks(self, msg: String) -> None:
        try:
            payload: dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f'Bad JSON: {e}')
            return

        self._tasks = payload.get('tasks', {})
        if self._state == AWAITING_TASKING and self._tasks:
            self._active_task = next(iter(self._tasks.values()))
            self._publish_state(ENROUTE_TO_TASKING)
        self._log_task_table()
        self._update_desired_heading()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _publish_state(self, state: str) -> None:
        if self._state == state:
            return
        self._state = state
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)
        self.get_logger().info(f'[{self._agent_id}] State -> {state}')

#---------------------------------------------------------------------------------------------------------------------------------------
    def _publish_velocity(self, velocity: float) -> None:
        msg = Float32()
        msg.data = float(velocity)
        self._vel_pub.publish(msg)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _state_decision(self) -> None:
        if self._lat is None or self._lon is None:
            return

        self.get_logger().info(
            f'[{self._agent_id}] STATE_CHECK state={self._state} '
            f'range={self._fuel_range:.2f} mi bingo={self._bingo:.2f} mi'
        )

        if self._bingo > 0.0 and self._fuel_range <= self._bingo and self._state not in (RTB, REFUELING):
            self._publish_state(RTB)
            self.get_logger().warning(f'[{self._agent_id}] BINGO! RANGE={self._fuel_range:.2f} mi <= '
                                     f'BINGO={self._bingo:.2f} mi — RETURNING TO HOMEPLATE')
            self._update_desired_heading()
            return
        
        if self._state == REFUELING:
            self._publish_velocity(0.0)
            if (self._refuel_started is not None and time.monotonic() - self._refuel_started >= _REFUEL_DURATION_SECONDS):
                self._refuel_started = None
                self._active_task = None
                self._publish_state(AWAITING_TASKING)
                self._publish_velocity(_DEFAULT_VELOCITY_MPH)
            self._update_desired_heading()
            return

        if self._state == AWAITING_TASKING:
            if self._tasks:
                self._active_task = next(iter(self._tasks.values()))
                self._publish_state(ENROUTE_TO_TASKING)
            self._update_desired_heading()
            return
        
        if self._state == ENROUTE_TO_TASKING:
            if(self._active_task):
                coords = self._active_task.get('coordinates', [])
                if len(coords) >= 3 and _point_in_polygon(self._lon, self._lat, coords):
                    self._publish_state(SEARCHING)
            else:
                self._publish_velocity(_DEFAULT_VELOCITY_MPH)
                self._update_desired_heading()
            return

        if self._state == RTB:
            distance = _haversine_miles(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon)
            if distance <= _HOMEPLATE_RADIUS_MILES:
                self._refuel_started = time.monotonic()
                self._publish_state(REFUELING)
                self._publish_velocity(0.0)
                self._update_desired_heading()
            else:
                self._update_desired_heading()
                

        if self._state in (AWAITING_TASKING, ENROUTE_TO_TASKING, SEARCHING):
            self._publish_velocity(_DEFAULT_VELOCITY_MPH)
        self._update_desired_heading()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _update_desired_heading(self) -> None:
        if self._lat is None:
            return

        # RTB overrides all mission tasks
        if self._state == RTB:
            hdg = _bearing(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon)
            logger_target = 'HOMEPLATE'
            self.get_logger().info(f'[{self._agent_id}] RTB heading → {hdg:.1f}° to HOMEPLATE')
            msg = Float32()
            msg.data = float(hdg)
            self._hdg_pub.publish(msg)
            return

        if not self._active_task:
            return

        best = self._active_task
        coords = best.get('coordinates', [])
        if not coords:
            return

        clat, clon = _centroid(coords)

        if self._state == SEARCHING:
            # orbit the polygon centroid in a clockwise circle around the NAI
            radial_hdg = _bearing(self._lat, self._lon, clat, clon)
            hdg = (radial_hdg + 90.0) % 360.0
            logger_target = f'{best.get("nai_name")} orbit centroid ({clat:.5f}, {clon:.5f})'
        else:
            hdg = _bearing(self._lat, self._lon, clat, clon)
            logger_target = f'{best.get("nai_name")} centroid ({clat:.5f}, {clon:.5f})'

        msg = Float32()
        msg.data = float(hdg)
        self._hdg_pub.publish(msg)
        self.get_logger().info(f'[{self._agent_id}] Desired heading → {hdg:.1f}° toward {logger_target}')

#---------------------------------------------------------------------------------------------------------------------------------------
    def _log_task_table(self) -> None:
        lines = [f'[{self._agent_id}] Task Table ({len(self._tasks)} NAIs):']
        for nai_name in sorted(self._tasks.keys()):
            t = self._tasks[nai_name]
            lines.append(
                f'  {nai_name}: priority={t.get("priority"):6s} '
                f'task_id={t.get("task_id")} '
                f'vertices={len(t.get("coordinates", []))}'
            )
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
