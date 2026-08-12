import json
import math
import random
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

_PRIORITY_RANK = {'High': 0, 'Medium': 1, 'Low': 2}  # kept for future use


def _centroid(coords: list) -> tuple[float, float]:
    """Return (lat, lon) centroid of [[lon, lat], ...] polygon vertices."""
    lons = [c[0] for c in coords]
    lats = [c[1] for c in coords]
    return sum(lats) / len(lats), sum(lons) / len(lons)


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return bearing in degrees (0=N, clockwise) from point 1 to point 2."""
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


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
        self._rtb: bool = False  # return to base flag

        ns = self._agent_id
        self._hdg_pub = self.create_publisher(Float32, f'{ns}/desired_heading', 10)

        self.create_subscription(String,    f'{ns}/tasks',           self._on_tasks,    10)
        self.create_subscription(NavSatFix, f'{ns}/drone/gps',       self._on_gps,      10)
        self.create_subscription(Float32,   f'{ns}/drone/fuel_range', self._on_fuel,    10)
        self.create_subscription(Float32,   f'{ns}/drone/bingo',     self._on_bingo,    10)
        self.create_subscription(String,    'swarm/homeplate',       self._on_homeplate, 10)

        self.get_logger().info(
            f'DecisionNode started — agent_id={self._agent_id}'
        )

    def _on_gps(self, msg: NavSatFix) -> None:
        self._lat = msg.latitude
        self._lon = msg.longitude

    def _on_fuel(self, msg: Float32) -> None:
        self._fuel_range = float(msg.data)
        self._check_bingo()

    def _on_bingo(self, msg: Float32) -> None:
        self._bingo = float(msg.data)
        self._check_bingo()

    def _on_homeplate(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._homeplate_lat = float(data['lat'])
            self._homeplate_lon = float(data['lon'])
        except (json.JSONDecodeError, KeyError):
            pass

    def _check_bingo(self) -> None:
        if self._fuel_range <= self._bingo and not self._rtb:
            self._rtb = True
            self.get_logger().warn(
                f'[{self._agent_id}] BINGO! RANGE={self._fuel_range:.2f} mi <= '
                f'BINGO={self._bingo:.2f} mi — RETURNING TO HOMEPLATE'
            )
            self._send_homeplate_heading()
        elif self._fuel_range > self._bingo and self._rtb:
            self._rtb = False
            self.get_logger().info(f'[{self._agent_id}] RANGE restored — resuming mission')

    def _on_tasks(self, msg: String) -> None:
        try:
            payload: dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Bad JSON: {e}')
            return

        self._tasks = payload.get('tasks', {})
        self._log_task_table()
        self._update_desired_heading()

    def _update_desired_heading(self) -> None:
        if self._lat is None:
            return

        # RTB overrides all mission tasks
        if self._rtb:
            self._send_homeplate_heading()
            return

        if not self._tasks:
            return

        best = random.choice(list(self._tasks.values()))
        coords = best.get('coordinates', [])
        if not coords:
            return

        clat, clon = _centroid(coords)
        hdg = _bearing(self._lat, self._lon, clat, clon)

        msg = Float32()
        msg.data = float(hdg)
        self._hdg_pub.publish(msg)
        self.get_logger().info(
            f'[{self._agent_id}] Desired heading → {hdg:.1f}° '
            f'toward {best.get("nai_name")} centroid ({clat:.5f}, {clon:.5f})'
        )

    def _send_homeplate_heading(self) -> None:
        if self._lat is None:
            return
        hdg = _bearing(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon)
        msg = Float32()
        msg.data = float(hdg)
        self._hdg_pub.publish(msg)
        self.get_logger().info(
            f'[{self._agent_id}] RTB heading → {hdg:.1f}° to HOMEPLATE'
        )

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
