import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from decision_pkg.utils import JsonLogger

class CommNode(Node):
    def __init__(self):
        super().__init__('comm_node')

        self.declare_parameter('agent_id', 'drone_001')
        self._agent_id: str = self.get_parameter('agent_id').get_parameter_value().string_value

        self._lat: float = 0.0
        self._lon: float = 0.0
        self._heading: float = 0.0
        self._velocity: float = 0.0
        self._fuel_range: float = 0.0
        self._bingo: float = 0.0
        self._state: str = 'AWAITING_TASKING'
        self._gps_ready: bool = False
        self._missions: dict[str, dict] = {}  # keyed by nai_name, latest mission wins

        status_topic = f'swarm/status/{self._agent_id}'
        self._pub = self.create_publisher(String, status_topic, 10)
        self._mission_pub = self.create_publisher(String, f'{self._agent_id}/missions', 10)
        self.logger = JsonLogger(name=f"{self._agent_id}", node='comm_node')
        ns = self._agent_id
        self.create_subscription(NavSatFix, f'{ns}/drone/gps',        self._on_gps,      10)
        self.create_subscription(Float32,   f'{ns}/drone/heading',    self._on_heading,  10)
        self.create_subscription(Float32,   f'{ns}/drone/velocity',   self._on_velocity, 10)
        self.create_subscription(Float32,   f'{ns}/drone/fuel_range', self._on_fuel,     10)
        self.create_subscription(Float32,   f'{ns}/drone/bingo',      self._on_bingo,    10)
        self.create_subscription(String,    f'{ns}/state',            self._on_state,     10)
        self.create_subscription(String,    'swarm/missions',         self._on_tasking,  10)
        self.create_timer(5.0, self._publish)

        self.logger.log("Successfully Started")

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_gps(self, msg: NavSatFix) -> None:
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._gps_ready = True
        
#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_heading(self, msg: Float32) -> None:
        self._heading = float(msg.data)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_velocity(self, msg: Float32) -> None:
        self._velocity = float(msg.data)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_fuel(self, msg: Float32) -> None:
        self._fuel_range = float(msg.data)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_bingo(self, msg: Float32) -> None:
        self._bingo = float(msg.data)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_state(self, msg: String) -> None:
        self._state = msg.data

#---------------------------------------------------------------------------------------------------------------------------------------
    def _publish(self) -> None:
        if not self._gps_ready:
            return
        payload = {
            'agent_id': self._agent_id,
            'lat': self._lat,
            'lon': self._lon,
            'heading': self._heading,
            'velocity': self._velocity,
            'fuel_range': self._fuel_range,
            'bingo': self._bingo,
            'state': self._state,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
        }
        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_tasking(self, msg: String) -> None:
        try:
            mission: dict = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        nai_name: str = mission.get('nai_name', mission.get('mission_name', 'Unknown'))
        is_update = nai_name in self._missions
        self._missions[nai_name] = mission

        action = 'Updated' if is_update else 'New'
        mission_id = mission.get('mission_id', mission.get('task_id', 'unknown'))
        self.get_logger().info(
            f'[{self._agent_id}] {action} mission — {nai_name} '
            f'({mission.get("priority")}, {mission.get("purpose")}, {mission.get("status")}) '
            f'id={mission_id}'
        )
        self._publish_task_table()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _publish_task_table(self) -> None:
        payload = {
            'agent_id': self._agent_id,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'missions': self._missions,
            'tasks': self._missions,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._mission_pub.publish(msg)

#---------------------------------------------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
