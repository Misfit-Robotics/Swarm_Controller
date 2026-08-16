import json
import time
from typing import Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_STATUS_PREFIX = '/swarm/status/'
_HEARTBEAT_TIMEOUT = 3.0
_DISCOVERY_PERIOD = 1.0
_HEARTBEAT_PERIOD = 2.0
HOMEPLATE_LAT = 30.5138   # Niceville, FL
HOMEPLATE_LON = -86.4869


class SwarmControllerCore(Node):
    """Core swarm controller logic (no output formatting)."""
    
    def __init__(self):
        super().__init__('swarm_controller')
        self._fleet: dict[str, dict[str, Any]] = {}
        self._subs: dict[str, Any] = {}
        self._task_counter: int = 0
        self._task_pub = self.create_publisher(String, 'swarm/tasking', 10)
        self._homeplate_pub = self.create_publisher(String, 'swarm/homeplate', 10)
        self.create_timer(_DISCOVERY_PERIOD, self._discover_drones)
        self.create_timer(_HEARTBEAT_PERIOD, self._check_heartbeats)
        # publish homeplate repeatedly so late-joining nodes receive it
        self.create_timer(5.0, self._publish_homeplate)
        self._publish_homeplate()

    def _discover_drones(self) -> None:
        topics = self.get_topic_names_and_types()
        status_topics = [t for t, _ in topics if t.startswith(_STATUS_PREFIX)]
        for topic in status_topics:
            if topic not in self._subs:
                self._subs[topic] = self.create_subscription(
                    String, topic,
                    lambda msg, t=topic: self._on_status(msg, t),
                    10,
                )
                agent_id = topic[len(_STATUS_PREFIX):]
                self.get_logger().info(f'Discovered drone: {agent_id}')

    def _on_status(self, msg: String, topic: str) -> None:
        try:
            data: dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f'Bad JSON on {topic}: {e}')
            return
        agent_id: str = data.get('agent_id', topic[len(_STATUS_PREFIX):])
        now = time.monotonic()
        was_online = self._fleet.get(agent_id, {}).get('online', False)
        self._fleet[agent_id] = {'last_seen': now, 'data': data, 'online': True}
        if not was_online:
            self.get_logger().info(f'[{agent_id}] ONLINE')

    def _check_heartbeats(self) -> None:
        now = time.monotonic()
        for agent_id, entry in self._fleet.items():
            age = now - entry['last_seen']
            if entry['online'] and age > _HEARTBEAT_TIMEOUT:
                entry['online'] = False
                self.get_logger().warning(f'[{agent_id}] OFFLINE')

    def get_fleet_snapshot(self) -> dict[str, dict[str, Any]]:
        """Return a snapshot of the current fleet state."""
        return dict(self._fleet)

    def _publish_homeplate(self) -> None:
        msg = String()
        msg.data = json.dumps({'lat': HOMEPLATE_LAT, 'lon': HOMEPLATE_LON})
        self._homeplate_pub.publish(msg)

    def publish_task(self, nai_name: str, priority: str, coordinates: list) -> str:
        """Publish an NAI tasking to all agents; returns the task_id."""
        self._task_counter += 1
        task_id = f'task_{self._task_counter:04d}'
        payload = {
            'task_id': task_id,
            'nai_name': nai_name,
            'priority': priority,
            'coordinates': coordinates,  # [[lon, lat], ...]
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._task_pub.publish(msg)
        self.get_logger().info(
            f'Task published: {task_id} — {nai_name} ({priority}), '
            f'{len(coordinates)} vertices'
        )
        return task_id
