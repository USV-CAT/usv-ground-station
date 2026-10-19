"""Marine units.

Everything inside the program is SI — metres, metres per second, degrees — and
is converted to what a helmsman reads (knots, nautical miles) only for display.
"""

from __future__ import annotations

#: Metres per second to knots.
MS_TO_KN = 3600.0 / 1852.0
#: One nautical mile in metres.
NM_M = 1852.0


def knots(speed_ms: float | None) -> float | None:
    return None if speed_ms is None else speed_ms * MS_TO_KN


def from_knots(speed_kn: float) -> float:
    return speed_kn / MS_TO_KN


def format_distance(metres: float | None) -> str:
    """Short distances in metres, long ones in nautical miles, as charts do."""
    if metres is None:
        return "—"
    if metres < 1000.0:
        return f"{metres:.0f} m"
    return f"{metres / NM_M:.2f} nm"


def format_duration(seconds: float | None) -> str:
    if seconds is None or seconds != seconds or seconds < 0:
        return "—"
    seconds = int(round(seconds))
    hours, rest = divmod(seconds, 3600)
    minutes, secs = divmod(rest, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


def format_bearing(degrees: float | None) -> str:
    return "—" if degrees is None else f"{degrees % 360.0:03.0f}°"


def compass_point(degrees: float) -> str:
    points = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return points[int(((degrees % 360.0) + 22.5) // 45.0) % 8]
