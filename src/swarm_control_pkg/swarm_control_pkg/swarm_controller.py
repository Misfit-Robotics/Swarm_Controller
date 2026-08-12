import time
import rclpy
from swarm_control_pkg.swarm_core import SwarmControllerCore

_STATUS_PERIOD = 5.0  # how often to print fleet status


class SwarmController(SwarmControllerCore):
    """ROS2 node with periodic status output."""
    
    def __init__(self):
        super().__init__()
        self.create_timer(_STATUS_PERIOD, self._print_status)
        self.get_logger().info('SwarmController started — waiting for drones…')

    def _print_status(self) -> None:
        fleet = self.get_fleet_snapshot()
        if not fleet:
            self.get_logger().info('Fleet status: no drones known')
            return
        lines = ['--- Fleet Status ---']
        now = time.monotonic()
        for agent_id in sorted(fleet.keys()):
            entry = fleet[agent_id]
            d = entry['data']
            state = 'ONLINE' if entry['online'] else 'OFFLINE'
            age = now - entry['last_seen']
            lines.append(
                f'  {agent_id}: {state} | '
                f'lat={d.get("lat", 0):.6f} lon={d.get("lon", 0):.6f} | '
                f'hdg={d.get("heading", 0):.1f}° vel={d.get("velocity", 0):.1f} mph | '
                f'age {age:.1f}s'
            )
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
