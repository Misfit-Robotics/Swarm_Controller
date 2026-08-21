import math


class enroute_policy:
    """Expert heading policy for enroute tasking. Inputs are the drone's current lat/lon and the centroid of the search area. The policy returns
       a compass heading in degrees that points directly at the centroid. This is deterministic and does not rely on learned parameters.
    """

    def __init__(self, *args, **kwargs):
        pass

#-----------------------------------------------------------------------------------------------------------------------------------------------    
    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Return bearing in degrees from point 1 to point 2."""
        dlat = lat2 - lat1
        dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
        heading = math.degrees(math.atan2(dlon, dlat))
        return (heading + 360.0) % 360.0


#-----------------------------------------------------------------------------------------------------------------------------------------------
    def get_action(self, current_lat: float, current_lon: float, centroid_lat: float, centroid_lon: float) -> float:
        """Return a heading in degrees toward the search-area centroid."""
        if current_lat is None or current_lon is None:
            raise ValueError('current_lat and current_lon are required')
        if centroid_lat is None or centroid_lon is None:
            raise ValueError('centroid_lat and centroid_lon are required')

        heading = self._bearing(float(current_lat), float(current_lon), float(centroid_lat), float(centroid_lon))
        return float(heading)