import math
import random


class find_policy:
    """Expert heading policy for the find/search state.

    The aircraft wanders randomly around the NAI centroid while remaining inside a
    bounded search radius. If the aircraft drifts outside that radius, it returns
    directly to the centroid to keep the search pattern contained.
    """

    def __init__(self, *args, **kwargs):
        self._rng = random.Random()

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

    def get_action(self, current_lat: float, current_lon: float, centroid_lat: float, centroid_lon: float) -> float:
        """Return a heading that keeps the aircraft randomly orbiting the NAI centroid."""
        if current_lat is None or current_lon is None:
            raise ValueError('current_lat and current_lon are required')
        if centroid_lat is None or centroid_lon is None:
            raise ValueError('centroid_lat and centroid_lon are required')

        current_lat = float(current_lat)
        current_lon = float(current_lon)
        centroid_lat = float(centroid_lat)
        centroid_lon = float(centroid_lon)

        max_radius_miles = 1.0
        distance_to_centroid = self._haversine_miles(current_lat, current_lon, centroid_lat, centroid_lon)

        # If the aircraft drifts outside the NAI boundary, steer it back to the centroid.
        if distance_to_centroid > max_radius_miles:
            heading = self._bearing(current_lat, current_lon, centroid_lat, centroid_lon)
            return float(heading)

        center_heading = self._bearing(current_lat, current_lon, centroid_lat, centroid_lon)
        offset_degrees = self._rng.uniform(-90.0, 90.0)
        heading = (center_heading + offset_degrees) % 360.0
        return float(heading)

    def select_action(self, current_lat: float, current_lon: float, centroid_lat: float, centroid_lon: float) -> float:
        """Compatibility wrapper matching the style used by the enroute policy."""
        return self.get_action(current_lat, current_lon, centroid_lat, centroid_lon)
