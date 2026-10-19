"""Offline vector map tiles.

A whole country's road, water and land-use geometry, built once by
``tools/osm_build.py`` into a tile index and a binary blob, then read lazily so
that only the tiles in view are decoded.  This matters more than it sounds: on
the water the ground station usually has no route to the internet, so a map
that fetches anything is a map that is blank exactly when it is needed.

Decoded tiles are cached with a least-recently-used policy.  Each line keeps
its size, so a zoomed-out view can leave out the ones smaller than a pixel:
there are thousands of them and none would show.  Zoomed out, the lines also
come from simplified copies of the tiles, with points under a pixel apart
merged.  A view zoomed
far enough out to cover more tiles than the cache can hold reports that rather
than attempting to draw a country's worth of polygons at once.
"""

from __future__ import annotations

import json
import logging
import math
from collections import OrderedDict
from pathlib import Path

import numpy as np
from PyQt5 import QtGui

from .polygons import polygon

log = logging.getLogger(__name__)

#: Above this many tiles in view, drawing is refused and the caller is told to
#: zoom in. Decoding more would stall the interface for seconds.
MAX_VISIBLE_TILES = 2500
#: Simplified copies of the tiles, for zoomed-out views: points closer than
#: this to the one before are dropped.  A view takes the coarsest one that
#: stays under a pixel.
SIMPLIFY_DEG = (0.0001, 0.0002, 0.0004)


def simplify(lat: np.ndarray, lon: np.ndarray, counts: np.ndarray,
             tolerance_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Drop each point in the same ``tolerance_deg`` cell as the one before.

    Every line keeps its two ends.  Done for a whole tile at once.
    """
    cell_lat = np.floor(lat / tolerance_deg)
    cell_lon = np.floor(lon / tolerance_deg)
    keep = np.empty(len(lat), bool)
    keep[0] = True
    keep[1:] = (cell_lat[1:] != cell_lat[:-1]) | (cell_lon[1:] != cell_lon[:-1])
    starts = np.cumsum(counts) - counts
    keep[starts] = True
    keep[starts + counts - 1] = True
    line = np.repeat(np.arange(len(counts)), counts)
    return lat[keep], lon[keep], np.bincount(line[keep], minlength=len(counts))


class TiledMap:
    """Reads the output of the map builder and serves polylines for a viewport."""

    def __init__(self, cache_tiles: int = 2000) -> None:
        self.tile_size_deg = 0.05
        self.bounds = [55.6, 20.9, 58.1, 28.3]
        self.layers: list[str] = []
        self.tiles: dict[str, dict] = {}
        self.source: Path | None = None

        self._blob = None
        self._cache: OrderedDict[tuple[int, int, str], tuple[list[QtGui.QPolygonF], np.ndarray]] = \
            OrderedDict()
        self._cache_limit = cache_tiles

    @property
    def is_loaded(self) -> bool:
        return bool(self.tiles)

    def load(self, index_path: str | Path) -> int:
        """Load a map index.  Returns the number of tiles available."""
        index_path = Path(index_path)
        data = json.loads(index_path.read_text(encoding="utf-8"))

        self.tile_size_deg = data["tile"]
        self.bounds = data["bbox"]
        self.layers = data["layers"]
        self.tiles = data["tiles"]

        blob_path = index_path.parent / data["bin"]
        if self._blob is not None:
            self._blob.close()
        self._blob = blob_path.open("rb")
        self._cache.clear()
        self.source = index_path
        return len(self.tiles)

    def close(self) -> None:
        if self._blob is not None:
            self._blob.close()
            self._blob = None
        self._cache.clear()
        self.tiles = {}

    # ------------------------------------------------------------------

    def _read(self, ix: int, iy: int, layer: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Every point of a tile's layer: latitudes, longitudes, and the count in each line."""
        empty = np.zeros(0), np.zeros(0), np.zeros(0, np.int64)
        record = self.tiles.get(f"{ix}_{iy}")
        if not record or layer not in record or self._blob is None:
            return empty
        offset, length = record[layer]
        self._blob.seek(offset)
        buffer = self._blob.read(length)
        # Each line is a uint16 count then that many (lat, lon) int32 pairs.
        # The loop only hops from count to count, as lean as Python allows:
        # there can be a hundred thousand lines in a wide view.
        headers: list[int] = []
        append = headers.append
        position, end = 0, len(buffer)
        while position + 2 <= end:
            following = position + 2 + 8 * (buffer[position] | buffer[position + 1] << 8)
            if following > end:
                log.debug("truncated tile %s_%s layer %s", ix, iy, layer)
                break
            append(position)
            position = following
        if not headers:
            return empty
        at = np.asarray(headers)
        lines = (np.append(at[1:], position) - at - 2) // 8
        # Cut the counts out and what is left is one run of points.
        keep = np.ones(position, bool)
        keep[at] = keep[at + 1] = False
        points = np.frombuffer(buffer, np.uint8, position)[keep].view("<i4").reshape(-1, 2)
        return points[:, 0] * 1e-7, points[:, 1] * 1e-7, lines[lines > 0]

    def _decode_tile(self, ix: int, iy: int, layer: str,
                     tolerance_deg: float = 0.0) -> tuple[list[QtGui.QPolygonF], np.ndarray]:
        """The tile's lines, and how big each one is in degrees."""
        key = (ix, iy, layer, tolerance_deg)
        cached = self._cache.get(key)
        if cached is not None:
            self._cache.move_to_end(key)
            return cached

        lat, lon, counts = self._read(ix, iy, layer)
        if tolerance_deg > 0.0 and len(counts):
            lat, lon, counts = simplify(lat, lon, counts, tolerance_deg)
        starts = np.cumsum(counts) - counts
        if len(counts):
            extents = np.maximum(
                np.maximum.reduceat(lat, starts) - np.minimum.reduceat(lat, starts),
                np.maximum.reduceat(lon, starts) - np.minimum.reduceat(lon, starts),
            )
        else:
            extents = np.zeros(0)
        if tolerance_deg > 0.0:
            # A simplified copy is only drawn zoomed out, where lines smaller
            # than the tolerance are under a pixel.
            big = extents >= tolerance_deg
            starts, counts, extents = starts[big], counts[big], extents[big]
        # Points are stored (latitude, longitude); the canvas draws in
        # (longitude, latitude) so that x is east.  One polygon of every point,
        # sliced in C++, is far quicker than one filled from numpy per line.
        whole = polygon(lon, lat)
        polygons = [whole.mid(a, n) for a, n in zip(starts.tolist(), counts.tolist(), strict=True)]

        entry = (polygons, extents)
        self._cache[key] = entry
        if len(self._cache) > self._cache_limit:
            self._cache.popitem(last=False)
        return entry

    def visible(
        self,
        min_lat: float,
        min_lon: float,
        max_lat: float,
        max_lon: float,
        layers: list[str],
        min_extent_deg: float = 0.0,
        tolerance_deg: float = 0.0,
    ) -> dict[str, list[QtGui.QPolygonF]] | None:
        """Geometry within the given bounds, or None if the view is too wide.

        Lines smaller than ``min_extent_deg`` across are left out, and with a
        ``tolerance_deg`` (one of :data:`SIMPLIFY_DEG`) the rest are simplified.
        """
        if not self.tiles:
            return None

        size = self.tile_size_deg
        ix0, ix1 = math.floor(min_lat / size), math.floor(max_lat / size)
        iy0, iy1 = math.floor(min_lon / size), math.floor(max_lon / size)
        if (ix1 - ix0 + 1) * (iy1 - iy0 + 1) > MAX_VISIBLE_TILES:
            return None

        result: dict[str, list[QtGui.QPolygonF]] = {layer: [] for layer in layers}
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                if f"{ix}_{iy}" not in self.tiles:
                    continue
                for layer in layers:
                    polygons, extents = self._decode_tile(ix, iy, layer, tolerance_deg)
                    if min_extent_deg > 0.0:
                        result[layer].extend(polygons[i] for i in np.flatnonzero(extents >= min_extent_deg))
                    else:
                        result[layer].extend(polygons)
        return result


def find_map_index(*candidates: str | Path) -> Path | None:
    """The first map index that exists, searching the usual places."""
    project_maps = Path(__file__).resolve().parents[3] / "maps"
    search: list[Path] = [Path(c).expanduser() for c in candidates if c]
    for folder in (Path("maps"), project_maps):
        search.append(folder / "latvia_map.idx.json")
        if folder.is_dir():
            search += sorted(folder.glob("*.idx.json"))
    for path in search:
        if path.exists():
            return path
    return None
