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
        self._mission_counter: int = 0
        self._active_mission: dict[str, Any] | None = None
        self._mission_pub = self.create_publisher(String, 'swarm/missions', 10)
        self._homeplate_pub = self.create_publisher(String, 'swarm/homeplate', 10)
        self.create_timer(_DISCOVERY_PERIOD, self._discover_drones)
        self.create_timer(_HEARTBEAT_PERIOD, self._check_heartbeats)
        # publish homeplate repeatedly so late-joining nodes receive it
        self.create_timer(5.0, self._publish_homeplate)
        # re-broadcast the active mission regularly so late-joining agents receive it
        self.create_timer(3.0, self._broadcast_active_mission)
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

    def _broadcast_active_mission(self) -> None:
        if self._active_mission is None:
            return
        msg = String()
        msg.data = json.dumps(self._active_mission)
        self._mission_pub.publish(msg)

    def publish_mission(self, nai_name: str, priority: str, purpose: str, coordinates: list, status: str = 'initiated') -> str:
        """Publish a mission to all agents; returns the mission_id."""
        self._mission_counter += 1
        mission_id = f'mission_{self._mission_counter:04d}'
        payload = {
            'mission_id': mission_id,
            'task_id': mission_id,
            'nai_name': nai_name,
            'mission_name': nai_name,
            'priority': priority,
            'purpose': purpose,
            'status': status,
            'coordinates': coordinates,  # [[lon, lat], ...]
            'timestamp': time.time(),
        }
        self._active_mission = payload
        msg = String()
        msg.data = json.dumps(payload)
        self._mission_pub.publish(msg)
        self.get_logger().info(
            f'Mission published: {mission_id} — {nai_name} ({priority}, {purpose}, {status}), '
            f'{len(coordinates)} vertices'
        )
        return mission_id

    def publish_task(self, nai_name: str, priority: str, coordinates: list) -> str:
        """Backward-compatible wrapper for legacy tasking callers."""
        return self.publish_mission(nai_name, priority, 'Find Targets', coordinates, 'initiated')
