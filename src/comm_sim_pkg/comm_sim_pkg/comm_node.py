import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String


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
        self._gps_ready: bool = False
        self._tasks: dict[str, dict] = {}  # keyed by nai_name, latest task wins

        status_topic = f'swarm/status/{self._agent_id}'
        self._pub = self.create_publisher(String, status_topic, 10)
        self._task_pub = self.create_publisher(String, f'{self._agent_id}/tasks', 10)

        ns = self._agent_id
        self.create_subscription(NavSatFix, f'{ns}/drone/gps',        self._on_gps,      10)
        self.create_subscription(Float32,   f'{ns}/drone/heading',    self._on_heading,  10)
        self.create_subscription(Float32,   f'{ns}/drone/velocity',   self._on_velocity, 10)
        self.create_subscription(Float32,   f'{ns}/drone/fuel_range', self._on_fuel,     10)
        self.create_subscription(Float32,   f'{ns}/drone/bingo',      self._on_bingo,    10)
        self.create_subscription(String,    'swarm/tasking',          self._on_tasking,  10)

        self.get_logger().info(f'CommNode started — agent_id={self._agent_id}, publishing to {status_topic}')

    def _on_gps(self, msg: NavSatFix) -> None:
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._gps_ready = True
        self._publish()
        self.get_logger().info(f'GPS: lat={self._lat:.6f} lon={self._lon:.6f}')

    def _on_heading(self, msg: Float32) -> None:
        self._heading = float(msg.data)
        self._publish()

    def _on_velocity(self, msg: Float32) -> None:
        self._velocity = float(msg.data)
        self._publish()

    def _on_fuel(self, msg: Float32) -> None:
        self._fuel_range = float(msg.data)

    def _on_bingo(self, msg: Float32) -> None:
        self._bingo = float(msg.data)

    def _publish(self) -> None:
        if not self._gps_ready:
            self.get_logger().debug('Waiting for GPS…')
            return
        payload = {
            'agent_id': self._agent_id,
            'lat': self._lat,
            'lon': self._lon,
            'heading': self._heading,
            'velocity': self._velocity,
            'fuel_range': self._fuel_range,
            'bingo': self._bingo,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
        }
        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)

    def _on_tasking(self, msg: String) -> None:
        try:
            task: dict = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        nai_name: str = task.get('nai_name', 'Unknown')
        is_update = nai_name in self._tasks
        self._tasks[nai_name] = task

        action = 'Updated' if is_update else 'New'
        self.get_logger().info(
            f'[{self._agent_id}] {action} task — {nai_name} '
            f'({task.get("priority")}) id={task.get("task_id")}'
        )
        self._publish_task_table()
        self._log_task_table()

    def _publish_task_table(self) -> None:
        payload = {
            'agent_id': self._agent_id,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'tasks': self._tasks,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._task_pub.publish(msg)

    def _log_task_table(self) -> None:
        lines = [f'[{self._agent_id}] Task Table ({len(self._tasks)} NAIs):']
        for nai_name, t in sorted(self._tasks.items()):
            lines.append(
                f'  {nai_name}: priority={t.get("priority"):6s} '
                f'task_id={t.get("task_id")} '
                f'vertices={len(t.get("coordinates", []))}'
            )
        self.get_logger().info('\n'.join(lines))


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
