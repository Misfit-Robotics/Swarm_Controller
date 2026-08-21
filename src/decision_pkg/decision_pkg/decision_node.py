import json
import math
import time
from typing import Any
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from decision_pkg.enroute_policy import enroute_policy
from decision_pkg.awaiting_mission_policy import awaiting_mission_policy
import decision_pkg.utils
from decision_pkg.utils import JsonLogger
AWAITING_TASKING = 'AWAITING_TASKING'
ENROUTE_TO_TASKING = 'ENROUTE_TO_TASKING'
SEARCHING = 'SEARCHING'
RTB = 'RTB'
REFUELING = 'REFUELING'
_DEFAULT_VELOCITY_MPH = 15.0
_HOMEPLATE_RADIUS_MILES = 0.1
_REFUEL_DURATION_SECONDS = 60.0

#---------------------------------------------------------------------------------------------------------------------------------------

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
        self._last_env_obs: dict[str, float | str | None] | None = None
        self._last_heading: float | None = None
        self._logger = JsonLogger(name=f"{self._agent_id}", node='decision_node')
        self._enroute_policy = enroute_policy()
        self._awaiting_policy = awaiting_mission_policy()
        ns = self._agent_id
        self._hdg_pub = self.create_publisher(Float32, f'{ns}/desired_heading', 10)
        self._vel_pub = self.create_publisher(Float32, f'{ns}/desired_velocity', 10)
        self._state_pub = self.create_publisher(String, f'{ns}/state', 10)

        self.create_subscription(String,    f'{ns}/missions',        self._on_mission,    10)
        self.create_subscription(NavSatFix, f'{ns}/drone/gps',       self._on_gps,      10)
        self.create_subscription(Float32,   f'{ns}/drone/fuel_range', self._on_fuel,    10)
        self.create_subscription(Float32,   f'{ns}/drone/bingo',     self._on_bingo,    10)
        self.create_subscription(String,    'swarm/homeplate',       self._on_homeplate, 10)
        self.create_timer(10.0, self._select_task_policy)
        self._publish_state("AWAITING_TASKING")
        self._logger.log("Successfully Started")

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_gps(self, msg: NavSatFix) -> None:
        self._lat = msg.latitude
        self._lon = msg.longitude

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_fuel(self, msg: Float32) -> None:
        self._fuel_range = float(msg.data)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_bingo(self, msg: Float32) -> None:
        self._bingo = float(msg.data)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_homeplate(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._homeplate_lat = float(data['lat'])
            self._homeplate_lon = float(data['lon'])
        except (json.JSONDecodeError, KeyError):
            pass

#---------------------------------------------------------------------------------------------------------------------------------------
    def _on_mission(self, msg: String) -> None:
        try:
            payload: dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._logger.log("Error decoding mission message", error=str(e))
            return

        self._tasks = payload.get('tasks', {})
        if self._state == AWAITING_TASKING and self._tasks:
            self._active_task = next(iter(self._tasks.values()))
            self._publish_state(ENROUTE_TO_TASKING)
        self._log_task_table()

#---------------------------------------------------------------------------------------------------------------------------------------
    def _publish_state(self, state: str) -> None:
        if self._state == state:
            return
        self._state = state
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _publish_velocity(self, velocity: float) -> None:
        msg = Float32()
        msg.data = float(velocity)
        self._vel_pub.publish(msg)

#---------------------------------------------------------------------------------------------------------------------------------------
    def _calc_reward(self, previous_range: float, new_range: float) -> float:
        if new_range < previous_range:
            return 0.5
        if new_range > previous_range:
            return -1.0
        return 0.0

#---------------------------------------------------------------------------------------------------------------------------------------
    def _record_enroute_transition(self, observation: dict[str, float | str | None]) -> None:
        """Expert-system policy does not use replay memory, but we still keep the observation capture."""
        if self._state != ENROUTE_TO_TASKING:
            return

        self._last_env_obs = observation

#---------------------------------------------------------------------------------------------------------------------------------------
    def _capture_env_obs(self) -> dict[str, float | str | None]:
        """Capture the current environment observation for the DQN/controller.

        Includes the agent's current lat/lon, local x/y offset to the active NAI
        centroid, the distance from the agent to the NAI, the current fuel range,
        and the current mission state.
        """
        lat = self._lat
        lon = self._lon
        x = 0.0
        y = 0.0
        range_to_nai = 0.0

        if lat is not None and lon is not None:
            coords = []
            if self._active_task is not None:
                coords = self._active_task.get('coordinates', [])

            if len(coords) >= 3:
                clat, clon = _centroid(coords)
                x = lon - clon
                y = lat - clat
                range_to_nai = _haversine_miles(lat, lon, clat, clon)

        return {
            'state': self._state,
            'lat': lat,
            'lon': lon,
            'x': x,
            'y': y,
            'range_to_nai': range_to_nai,
            'fuel_range': self._fuel_range,
        }
#---------------------------------------------------------------------------------------------------------------------------------------
    def _update_desired_heading(self, hdg) -> None:
            self._last_heading = float(hdg)
            msg = Float32()
            msg.data = float(hdg)
            self._hdg_pub.publish(msg)
            return

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
        self._logger.log("Task Table", table='\n'.join(lines))

#---------------------------------------------------------------------------------------------------------------------------------------
    def _select_task_policy(self) -> None:
        if self._lat is None or self._lon is None:
            return

        if self._bingo > 0.0 and self._fuel_range <= self._bingo and self._state not in (RTB, REFUELING):
            self._publish_state(RTB)
            self._update_desired_heading(self._enroute_policy.get_action(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon))
            return
        
        if self._state == REFUELING:
            self._publish_velocity(0.0)
            if (self._refuel_started is not None and time.monotonic() - self._refuel_started >= _REFUEL_DURATION_SECONDS):
                self._refuel_started = None
                self._active_task = None
                self._publish_state(AWAITING_TASKING)
                self._publish_velocity(_DEFAULT_VELOCITY_MPH)
            self._update_desired_heading(self._refueling_policy.get_action(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon))
            return

        if self._state == AWAITING_TASKING:
            if self._tasks:
                self._active_task = next(iter(self._tasks.values()))
                self._publish_state(ENROUTE_TO_TASKING)
            self._update_desired_heading(self._awaiting_policy.get_action(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon))
            return
        
        if self._state == ENROUTE_TO_TASKING:
            if(self._active_task):
                coords = self._active_task.get('coordinates', [])
                if len(coords) >= 3 and _point_in_polygon(self._lon, self._lat, coords):
                    self._publish_state(SEARCHING)
                    return
            self._update_desired_heading(self._enroute_tasking_policy.get_action(self._lat, self._lon, coords[0], coords[1]))
            return

        if self._state == RTB:
            distance = _haversine_miles(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon)
            if distance <= _HOMEPLATE_RADIUS_MILES:
                self._refuel_started = time.monotonic()
                self._publish_state(REFUELING)
                self._publish_velocity(0.0)
                self._update_desired_heading(self._rtb_policy.get_action(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon))
            else:
                self._update_desired_heading(self._rtb_policy.get_action(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon))
                

        if self._state in (AWAITING_TASKING, ENROUTE_TO_TASKING, SEARCHING):
            self._publish_velocity(_DEFAULT_VELOCITY_MPH)
        self._update_desired_heading(self._searching_policy.get_action(self._lat, self._lon, self._homeplate_lat, self._homeplate_lon))

        current_obs = self._capture_env_obs()
        if self._state == ENROUTE_TO_TASKING:
            self._record_enroute_transition(current_obs)

        self._logger.log("State Update", state=self._state, lat=self._lat, lon=self._lon, fuel_range=self._fuel_range, bingo=self._bingo, heading=self._last_heading, active_task=self._active_task)

#---------------------------------------------------------------------------------------------------------------------------------------
 
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
