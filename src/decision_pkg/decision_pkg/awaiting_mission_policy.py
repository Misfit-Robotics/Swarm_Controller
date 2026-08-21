import math


class awaiting_mission_policy:
    """Expert heading policy for the awaiting-mission state.

    The aircraft holds an orbit around homeplate at roughly 1 mile radius while
    waiting for an active mission. The logic mirrors the structure of the
    enroute policy: inputs are the current lat/lon and homeplate lat/lon, then a
    deterministic heading is returned in compass degrees.
    """

    def __init__(self, *args, **kwargs):
        pass

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Return bearing in degrees from point 1 to point 2."""
        dlat = lat2 - lat1
        dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
        heading = math.degrees(math.atan2(dlon, dlat))
        return (heading + 360.0) % 360.0

    @staticmethod
    def _haversine_miles(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Return great-circle distance in miles between two lat/lon points."""
        radius_miles = 3958.8
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (
            math.sin(dlat / 2.0) ** 2
            + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2.0) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return radius_miles * c

    def get_action(self, current_lat: float, current_lon: float, homeplate_lat: float, homeplate_lon: float) -> float:
        """Return a heading that keeps the aircraft orbiting homeplate at ~1 mile radius."""
        if current_lat is None or current_lon is None:
            raise ValueError('current_lat and current_lon are required')
        if homeplate_lat is None or homeplate_lon is None:
            raise ValueError('homeplate_lat and homeplate_lon are required')

        current_lat = float(current_lat)
        current_lon = float(current_lon)
        homeplate_lat = float(homeplate_lat)
        homeplate_lon = float(homeplate_lon)

        distance_miles = self._haversine_miles(current_lat, current_lon, homeplate_lat, homeplate_lon)
        target_radius_miles = 1.0
        orbit_direction = 90.0

        if distance_miles > target_radius_miles + 0.25:
            # Too far out: steer back toward homeplate.
            heading = self._bearing(current_lat, current_lon, homeplate_lat, homeplate_lon)
        elif distance_miles < target_radius_miles - 0.25:
            # Too close: steer away from homeplate.
            heading = (self._bearing(current_lat, current_lon, homeplate_lat, homeplate_lon) + 180.0) % 360.0
        else:
            # In the desired orbit band: hold a clockwise tangent to the 1-mile orbit.
            radial_heading = self._bearing(current_lat, current_lon, homeplate_lat, homeplate_lon)
            heading = (radial_heading + orbit_direction) % 360.0

        return float(heading)

    def select_action(self, current_lat: float, current_lon: float, homeplate_lat: float, homeplate_lon: float) -> float:
        """Compatibility wrapper matching the style used by the enroute policy."""
        return self.get_action(current_lat, current_lon, homeplate_lat, homeplate_lon)
