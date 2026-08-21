import json
import time
from datetime import datetime
from pathlib import Path
import math

#---------------------------------------------------------------------------------------------------------------------------------------
def _centroid(coords: list) -> tuple[float, float]:
    """Return (lat, lon) centroid of [[lon, lat], ...] polygon vertices."""
    lons = [c[0] for c in coords]
    lats = [c[1] for c in coords]
    return sum(lats) / len(lats), sum(lons) / len(lons)

#---------------------------------------------------------------------------------------------------------------------------------------
def _point_in_polygon(lon: float, lat: float, coords: list) -> bool:
    """Return whether a [lon, lat] point is inside a polygon."""
    inside = False
    previous = coords[-1]
    for current in coords:
        current_lon, current_lat = current[0], current[1]
        previous_lon, previous_lat = previous[0], previous[1]
        crosses_latitude = (current_lat > lat) != (previous_lat > lat)
        if crosses_latitude:
            crossing_lon = ((previous_lon - current_lon) * (lat - current_lat)
                            / (previous_lat - current_lat) + current_lon)
            if lon < crossing_lon:
                inside = not inside
        previous = current
    return inside

#---------------------------------------------------------------------------------------------------------------------------------------
def _haversine_miles(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    earth_radius_miles = 3958.8
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return earth_radius_miles * 2 * math.asin(math.sqrt(a))

#---------------------------------------------------------------------------------------------------------------------------------------
class JsonLogger:
    def __init__(self, name: str, node: str, base_dir: str = "log"):
        # Create timestamped folder: YYYY-MM-DD_HH-MM-SS
        timestamp = datetime.now().strftime("%Y-%m-%d")
        log_dir = Path(base_dir) / timestamp
        log_dir.mkdir(parents=True, exist_ok=True)
        self.name = name
        self.node = node
        self.file = open(log_dir / f"{name}.jsonl", "a", buffering=1)

    def log(self, event: str, **fields):
        entry = {
            "ts": time.time(),
            "agent": self.name,
            "node": self.node,
            "event": event,
            **fields
        }
        self.file.write(json.dumps(entry) + "\n")

    def close(self):
        self.file.close()