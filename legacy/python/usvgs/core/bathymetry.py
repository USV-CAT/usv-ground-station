"""Water depth: the chart, and what the boat has measured.

Two different things, kept apart because they deserve different trust:

* :class:`DepthGrid` is the *chart* — a regular latitude/longitude grid loaded
  from a file before the trip.  The bundled one is EMODnet's Gulf of Riga
  digital terrain model, about 115 m per cell.  Inland lakes carry only a
  placeholder there, so the builder estimated their depth from the distance to
  the shore and the grid marks those cells as estimated.
* :class:`SoundingGrid` is the *survey* — what the boat's echo sounder actually
  read, binned into small square cells as the boat crosses them.  This is the
  product of a survey and is what gets exported.

Depths are metres, positive down.  Land and unknown cells are NaN.
"""

from __future__ import annotations

import csv
import math
import struct
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import geo
from .geo import LatLon

#: Quality flags stored alongside a chart grid.
MEASURED = 0
ESTIMATED = 1


class DepthGrid:
    """A depth chart on a regular grid.  Row 0 is the northern edge."""

    def __init__(
        self,
        depth: np.ndarray,
        north: float,
        west: float,
        dlat: float,
        dlon: float,
        quality: np.ndarray | None = None,
        source: str = "",
    ) -> None:
        self.depth = np.asarray(depth, dtype=np.float32)
        self.north = float(north)
        self.west = float(west)
        self.dlat = float(dlat)
        self.dlon = float(dlon)
        rows, cols = self.depth.shape
        self.quality = (
            np.zeros((rows, cols), np.uint8) if quality is None else np.asarray(quality, np.uint8)
        )
        self.source = source

    # -- geometry -----------------------------------------------------------

    @property
    def shape(self) -> tuple[int, int]:
        return self.depth.shape

    @property
    def south(self) -> float:
        return self.north - self.shape[0] * self.dlat

    @property
    def east(self) -> float:
        return self.west + self.shape[1] * self.dlon

    @property
    def bounds(self) -> geo.BBox:
        return geo.BBox(self.south, self.west, self.north, self.east)

    def contains(self, lat: float, lon: float) -> bool:
        return self.south <= lat < self.north and self.west <= lon < self.east

    def _fractional(self, lat: float, lon: float) -> tuple[float, float]:
        """Row and column of a position, measured between cell centres."""
        return (self.north - lat) / self.dlat - 0.5, (lon - self.west) / self.dlon - 0.5

    # -- sampling -----------------------------------------------------------

    def depth_at(self, lat: float, lon: float) -> float:
        """Interpolated depth, or NaN on land or off the grid.

        Bilinear between the four surrounding cells.  When some of them are
        land, the water cells are blended and the depth tapers to zero as the
        position approaches the land, so the shore is a slope rather than a
        cliff.
        """
        if not self.contains(lat, lon):
            return math.nan
        rows, cols = self.shape
        fr, fc = self._fractional(lat, lon)
        r0 = int(math.floor(fr))
        c0 = int(math.floor(fc))
        tr, tc = fr - r0, fc - c0
        total = 0.0
        weight_water = 0.0
        weight_all = 0.0
        for dr, wr in ((0, 1.0 - tr), (1, tr)):
            for dc, wc in ((0, 1.0 - tc), (1, tc)):
                r = min(rows - 1, max(0, r0 + dr))
                c = min(cols - 1, max(0, c0 + dc))
                w = wr * wc
                weight_all += w
                value = self.depth[r, c]
                if value == value:
                    total += w * float(value)
                    weight_water += w
        if weight_water <= 0.0 or weight_water < 0.35 * weight_all:
            return math.nan
        # Blend towards zero by the land fraction, so depth shoals to the shore.
        return (total / weight_water) * (weight_water / weight_all)

    def depth_nearest(self, lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
        """Depth of the nearest cell for arrays of positions; NaN on land or off the grid."""
        rows = np.floor((self.north - np.asarray(lat)) / self.dlat).astype(int)
        cols = np.floor((np.asarray(lon) - self.west) / self.dlon).astype(int)
        inside = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
        out = np.full(rows.shape, np.nan, np.float32)
        out[inside] = self.depth[rows[inside], cols[inside]]
        return out

    def quality_at(self, lat: float, lon: float) -> int:
        if not self.contains(lat, lon):
            return ESTIMATED
        fr, fc = self._fractional(lat, lon)
        r = min(self.shape[0] - 1, max(0, int(round(fr))))
        c = min(self.shape[1] - 1, max(0, int(round(fc))))
        return int(self.quality[r, c])

    def is_water(self, lat: float, lon: float) -> bool:
        value = self.depth_at(lat, lon)
        return value == value

    def deepest_near(self, lat: float, lon: float, radius_cells: int = 6) -> LatLon | None:
        """The deepest water cell centre near a position, for spawning a boat."""
        fr, fc = self._fractional(lat, lon)
        r0, c0 = int(round(fr)), int(round(fc))
        best = None
        for r in range(max(0, r0 - radius_cells), min(self.shape[0], r0 + radius_cells + 1)):
            for c in range(max(0, c0 - radius_cells), min(self.shape[1], c0 + radius_cells + 1)):
                value = self.depth[r, c]
                if value == value and (best is None or value > best[0]):
                    best = (value, r, c)
        if best is None:
            return None
        _, r, c = best
        return LatLon(self.north - (r + 0.5) * self.dlat, self.west + (c + 0.5) * self.dlon)

    # -- files --------------------------------------------------------------

    def save_npz(self, path: str | Path) -> None:
        # Decimetres in an int16 keep a whole gulf in about a megabyte.
        decimetres = np.where(np.isnan(self.depth), -32768, np.round(self.depth * 10.0))
        np.savez_compressed(
            path,
            depth_dm=decimetres.astype(np.int16),
            quality=self.quality,
            geo=np.array([self.north, self.west, self.dlat, self.dlon]),
            source=np.array(self.source),
        )

    @classmethod
    def load(cls, path: str | Path) -> DepthGrid:
        """Load any supported format, chosen by extension."""
        path = Path(path)
        suffix = path.suffix.lower()
        if suffix == ".npz":
            return cls.load_npz(path)
        if suffix == ".asc":
            return cls.load_ascii_grid(path)
        if suffix in (".tif", ".tiff"):
            return cls.load_geotiff(path)
        if suffix in (".xyz", ".csv", ".txt"):
            return cls.load_xyz(path)
        raise ValueError(f"unsupported depth grid format: {path.name}")

    @classmethod
    def load_npz(cls, path: str | Path) -> DepthGrid:
        with np.load(path) as data:
            decimetres = data["depth_dm"]
            depth = np.where(decimetres == -32768, np.nan, decimetres / 10.0)
            north, west, dlat, dlon = (float(v) for v in data["geo"])
            source = str(data["source"]) if "source" in data else Path(path).name
            return cls(depth, north, west, dlat, dlon, data["quality"], source)

    @classmethod
    def from_elevation(cls, elevation: np.ndarray, north: float, west: float,
                       dlat: float, dlon: float, source: str = "") -> DepthGrid:
        """From a terrain model where the sea floor is negative and land positive."""
        elevation = np.asarray(elevation, dtype=np.float32)
        depth = np.where(elevation < 0.0, -elevation, np.nan).astype(np.float32)
        return cls(depth, north, west, dlat, dlon, source=source)

    @classmethod
    def load_ascii_grid(cls, path: str | Path) -> DepthGrid:
        """ESRI ASCII grid, in geographic degrees, values as *elevation*."""
        header: dict[str, float] = {}
        with open(path, encoding="utf-8") as handle:
            while len(header) < 6:
                position = handle.tell()
                line = handle.readline()
                parts = line.split()
                if len(parts) != 2 or not parts[0][0].isalpha():
                    handle.seek(position)
                    break
                header[parts[0].lower()] = float(parts[1])
            values = np.loadtxt(handle, dtype=np.float32)
        cell = header["cellsize"]
        rows = int(header["nrows"])
        nodata = header.get("nodata_value")
        if nodata is not None:
            values[values == nodata] = np.nan
        south = header.get("yllcorner", header.get("yllcenter", 0.0) - cell / 2)
        west = header.get("xllcorner", header.get("xllcenter", 0.0) - cell / 2)
        return cls.from_elevation(values, south + rows * cell, west, cell, cell, Path(path).name)

    @classmethod
    def load_xyz(cls, path: str | Path, cell_deg: float = 0.0005) -> DepthGrid:
        """Scattered soundings ``lon, lat, depth`` binned onto a grid.

        Depth is taken as positive down, which is what survey exports (and
        :meth:`SoundingGrid.export_xyz`) write.
        """
        points = []
        with open(path, encoding="utf-8") as handle:
            for row in csv.reader(handle, delimiter=_sniff_delimiter(path)):
                try:
                    points.append((float(row[0]), float(row[1]), float(row[2])))
                except (ValueError, IndexError):
                    continue
        if not points:
            raise ValueError(f"no soundings in {path}")
        data = np.array(points, dtype=np.float64)
        lon, lat, depth = data[:, 0], data[:, 1], data[:, 2]
        west, north = lon.min() - cell_deg, lat.max() + cell_deg
        cols = int((lon.max() - west) / cell_deg) + 2
        rows = int((north - lat.min()) / cell_deg) + 2
        r = ((north - lat) / cell_deg).astype(int)
        c = ((lon - west) / cell_deg).astype(int)
        total = np.zeros((rows, cols))
        count = np.zeros((rows, cols))
        np.add.at(total, (r, c), depth)
        np.add.at(count, (r, c), 1)
        with np.errstate(invalid="ignore", divide="ignore"):
            grid = np.where(count > 0, total / count, np.nan)
        return cls(grid, north, west, cell_deg, cell_deg, source=Path(path).name)

    @classmethod
    def load_geotiff(cls, path: str | Path) -> DepthGrid:
        """A single-band float GeoTIFF in EPSG:4326 holding *elevation*."""
        elevation, north, west, dlat, dlon = read_geotiff(path)
        return cls.from_elevation(elevation, north, west, dlat, dlon, Path(path).name)


def _sniff_delimiter(path: str | Path) -> str:
    with open(path, encoding="utf-8") as handle:
        sample = handle.read(2048)
    for delimiter in (",", ";", "\t"):
        if delimiter in sample:
            return delimiter
    return " "


def read_geotiff(path: str | Path) -> tuple[np.ndarray, float, float, float, float]:
    """Read an uncompressed, single-band, 32-bit float GeoTIFF.

    Enough of TIFF to read what map servers such as EMODnet's WCS return,
    without pulling in GDAL.  Returns ``(values, north, west, dlat, dlon)``.
    """
    blob = Path(path).read_bytes()
    order = {b"II": "<", b"MM": ">"}.get(blob[:2])
    if order is None:
        raise ValueError("not a TIFF file")
    (offset,) = struct.unpack(order + "I", blob[4:8])
    (count,) = struct.unpack(order + "H", blob[offset:offset + 2])
    sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 11: 4, 12: 8, 16: 8}
    formats = {1: "B", 3: "H", 4: "I", 11: "f", 12: "d", 16: "Q"}
    tags: dict[int, list] = {}
    for index in range(count):
        entry = blob[offset + 2 + 12 * index: offset + 14 + 12 * index]
        tag, kind, number, raw = struct.unpack(order + "HHI4s", entry)
        size = sizes.get(kind, 1) * number
        payload = raw if size <= 4 else blob[struct.unpack(order + "I", raw)[0]:][:size]
        if kind in formats:
            tags[tag] = list(struct.unpack(order + formats[kind] * number, payload[:size]))

    width, height = tags[256][0], tags[257][0]
    if tags.get(259, [1])[0] != 1:
        raise ValueError("compressed TIFFs are not supported; request an uncompressed one")
    if tags.get(258, [32])[0] != 32 or tags.get(339, [3])[0] != 3:
        raise ValueError("only 32-bit float TIFFs are supported")
    dtype = np.dtype(order + "f4")
    values = np.full((height, width), np.nan, np.float32)

    if 322 in tags:  # tiled
        tile_w, tile_h = tags[322][0], tags[323][0]
        across = (width + tile_w - 1) // tile_w
        for index, (start, length) in enumerate(zip(tags[324], tags[325], strict=False)):
            tile = np.frombuffer(blob[start:start + length], dtype).reshape(tile_h, tile_w)
            row, col = divmod(index, across)
            y0, x0 = row * tile_h, col * tile_w
            h, w = min(tile_h, height - y0), min(tile_w, width - x0)
            values[y0:y0 + h, x0:x0 + w] = tile[:h, :w]
    else:  # stripped
        rows = [np.frombuffer(blob[s:s + n], dtype) for s, n in zip(tags[273], tags[279], strict=False)]
        values[:] = np.concatenate(rows)[: width * height].reshape(height, width)

    nodata = _ascii_tag(blob, order, offset, 42113)  # GDAL_NODATA, stored as text
    if nodata is not None:
        values[values == np.float32(nodata)] = np.nan

    scale = _double_tag(blob, order, offset, 33550)
    tie = _double_tag(blob, order, offset, 33922)
    transform = _double_tag(blob, order, offset, 34264)
    if transform:
        dlon, west, dlat, north = transform[0], transform[3], -transform[5], transform[7]
    elif scale and tie:
        dlon, dlat = scale[0], scale[1]
        west, north = tie[3] - tie[0] * dlon, tie[4] + tie[1] * dlat
    else:
        raise ValueError("TIFF has no georeferencing")
    return values, north, west, dlat, dlon


def _raw_tag(blob: bytes, order: str, offset: int, wanted: int) -> tuple[int, int, bytes] | None:
    (count,) = struct.unpack(order + "H", blob[offset:offset + 2])
    for index in range(count):
        entry = blob[offset + 2 + 12 * index: offset + 14 + 12 * index]
        tag, kind, number, raw = struct.unpack(order + "HHI4s", entry)
        if tag == wanted:
            size = {2: 1, 12: 8}.get(kind, 4) * number
            data = raw if size <= 4 else blob[struct.unpack(order + "I", raw)[0]:][:size]
            return kind, number, data
    return None


def _double_tag(blob: bytes, order: str, offset: int, wanted: int) -> list[float] | None:
    found = _raw_tag(blob, order, offset, wanted)
    if found is None or found[0] != 12:
        return None
    return list(struct.unpack(order + "d" * found[1], found[2]))


def _ascii_tag(blob: bytes, order: str, offset: int, wanted: int) -> float | None:
    found = _raw_tag(blob, order, offset, wanted)
    if found is None or found[0] != 2:
        return None
    try:
        return float(found[2].rstrip(b"\0").decode())
    except ValueError:
        return None


# --------------------------------------------------------------------------
# The survey
# --------------------------------------------------------------------------

#: Cells per side of one chunk of the sounding grid.  The map caches one image
#: per chunk and redraws only chunks whose cells changed.
CHUNK = 64


@dataclass
class Sounding:
    t: float
    lat: float
    lon: float
    depth_m: float


class SoundingGrid:
    """Echo-sounder readings, binned into square cells of ``cell_m`` metres.

    Cells are indexed in a local tangent plane about ``origin``, so a cell is
    the same size anywhere in the survey.  Each holds a running mean, and the
    raw soundings are kept as well for export.
    """

    def __init__(self, origin: LatLon, cell_m: float = 5.0) -> None:
        self.origin = origin
        self.cell_m = float(cell_m)
        self._sum: dict[tuple[int, int], float] = {}
        self._count: dict[tuple[int, int], int] = {}
        #: The cells of each chunk, so redrawing one need not probe all of it.
        self._by_chunk: dict[tuple[int, int], set[tuple[int, int]]] = {}
        self.soundings: list[Sounding] = []
        #: Chunks whose cells changed since the map last looked.
        self.dirty_chunks: set[tuple[int, int]] = set()
        self.min_depth = math.inf
        self.max_depth = 0.0

    def __len__(self) -> int:
        return len(self._count)

    def cell_of(self, lat: float, lon: float) -> tuple[int, int]:
        local = geo.to_metres(self.origin, (lat, lon))
        return math.floor(local.east / self.cell_m), math.floor(local.north / self.cell_m)

    def cell_centre(self, cell: tuple[int, int]) -> LatLon:
        return geo.to_lat_lon(
            self.origin,
            geo.Metres((cell[0] + 0.5) * self.cell_m, (cell[1] + 0.5) * self.cell_m),
        )

    def add(self, lat: float, lon: float, depth_m: float, swath_m: float = 0.0,
            t: float | None = None) -> int:
        """Record a sounding; credit it to every cell within half a swath.

        Returns how many cells were seen for the first time, which is what the
        coverage counter cares about.
        """
        if depth_m != depth_m:
            return 0
        self.soundings.append(Sounding(time.time() if t is None else t, lat, lon, depth_m))
        self.min_depth = min(self.min_depth, depth_m)
        self.max_depth = max(self.max_depth, depth_m)
        centre = self.cell_of(lat, lon)
        reach = int(max(0.0, swath_m / 2.0) // self.cell_m)
        new = 0
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                if dx * dx + dy * dy > reach * reach + reach:
                    continue
                cell = (centre[0] + dx, centre[1] + dy)
                chunk = (cell[0] // CHUNK, cell[1] // CHUNK)
                if cell not in self._count:
                    new += 1
                    self._count[cell] = 0
                    self._sum[cell] = 0.0
                    self._by_chunk.setdefault(chunk, set()).add(cell)
                self._count[cell] += 1
                self._sum[cell] += depth_m
                self.dirty_chunks.add(chunk)
        return new

    def depth_of(self, cell: tuple[int, int]) -> float | None:
        count = self._count.get(cell)
        return None if not count else self._sum[cell] / count

    def cells(self) -> Iterator[tuple[tuple[int, int], float]]:
        for cell, count in self._count.items():
            yield cell, self._sum[cell] / count

    def chunk_cells(self, chunk: tuple[int, int]) -> Iterator[tuple[int, int, float]]:
        """Cells of one chunk as ``(column, row-from-south, depth)`` within it."""
        x0, y0 = chunk[0] * CHUNK, chunk[1] * CHUNK
        for cell in self._by_chunk.get(chunk, ()):
            yield cell[0] - x0, cell[1] - y0, self._sum[cell] / self._count[cell]

    def chunks(self) -> set[tuple[int, int]]:
        return set(self._by_chunk)

    def chunk_bounds(self, chunk: tuple[int, int]) -> tuple[LatLon, LatLon]:
        """South-west and north-east corners of a chunk."""
        size = CHUNK * self.cell_m
        sw = geo.to_lat_lon(self.origin, geo.Metres(chunk[0] * size, chunk[1] * size))
        ne = geo.to_lat_lon(self.origin, geo.Metres((chunk[0] + 1) * size, (chunk[1] + 1) * size))
        return sw, ne

    def coverage(self, rect) -> float:
        """Fraction of a :class:`~usvgs.core.mission.Rectangle` that has been sounded."""
        if rect is None or rect.area_m2 <= 0:
            return 0.0
        total = max(1.0, rect.area_m2 / (self.cell_m * self.cell_m))
        inside = sum(1 for cell in self._count if rect.contains(self.cell_centre(cell)))
        return min(1.0, inside / total)

    def clear(self) -> None:
        self.dirty_chunks |= self.chunks()
        self._sum.clear()
        self._count.clear()
        self._by_chunk.clear()
        self.soundings.clear()
        self.min_depth = math.inf
        self.max_depth = 0.0

    def export_xyz(self, path: str | Path) -> int:
        """Raw soundings as ``lon,lat,depth_m,unix_time``.  Returns the row count."""
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["lon", "lat", "depth_m", "time"])
            for s in self.soundings:
                writer.writerow([f"{s.lon:.7f}", f"{s.lat:.7f}", f"{s.depth_m:.2f}", f"{s.t:.2f}"])
        return len(self.soundings)

    def export_grid_csv(self, path: str | Path) -> int:
        """The binned grid: one row per cell, ``lon,lat,depth_m,count``."""
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["lon", "lat", "depth_m", "count"])
            for cell, depth in self.cells():
                centre = self.cell_centre(cell)
                writer.writerow(
                    [f"{centre.lon:.7f}", f"{centre.lat:.7f}", f"{depth:.2f}", self._count[cell]]
                )
        return len(self._count)


# --------------------------------------------------------------------------
# Contours
# --------------------------------------------------------------------------


def contour_segments(grid: DepthGrid, level: float) -> np.ndarray:
    """Marching squares over the chart: the ``level`` isobath as line segments.

    Returns an ``(n, 4)`` array of ``lon0, lat0, lon1, lat1``.  Segments are not
    joined into lines; the map only strokes them, and unjoined is enough.
    Land counts as zero depth, so the zero contour is the coastline.
    """
    z = np.nan_to_num(grid.depth, nan=0.0)
    a, b = z[:-1, :-1], z[:-1, 1:]   # top-left, top-right
    c, d = z[1:, 1:], z[1:, :-1]     # bottom-right, bottom-left
    index = (
        (a > level).astype(np.uint8)
        | ((b > level).astype(np.uint8) << 1)
        | ((c > level).astype(np.uint8) << 2)
        | ((d > level).astype(np.uint8) << 3)
    )

    def lerp(v0, v1):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (level - v0) / (v1 - v0)
        return np.clip(np.nan_to_num(t, nan=0.5), 0.0, 1.0)

    rows, cols = np.indices(index.shape)
    # Edge crossing points in (row, col) grid space.
    top = (rows, cols + lerp(a, b))
    right = (rows + lerp(b, c), cols + 1)
    bottom = (rows + 1, cols + lerp(d, c))
    left = (rows + lerp(a, d), cols)
    edges = {"t": top, "r": right, "b": bottom, "l": left}
    table = {
        1: ["lt"], 2: ["tr"], 3: ["lr"], 4: ["rb"], 5: ["lt", "rb"], 6: ["tb"],
        7: ["lb"], 8: ["bl"], 9: ["tb"], 10: ["tr", "bl"], 11: ["rb"], 12: ["lr"],
        13: ["tr"], 14: ["lt"],
    }
    pieces = []
    for case, pairs in table.items():
        mask = index == case
        if not mask.any():
            continue
        for pair in pairs:
            p0, p1 = edges[pair[0]], edges[pair[1]]
            r0, c0 = p0[0][mask], p0[1][mask]
            r1, c1 = p1[0][mask], p1[1][mask]
            pieces.append(np.stack([c0, r0, c1, r1], axis=1))
    if not pieces:
        return np.zeros((0, 4))
    seg = np.concatenate(pieces).astype(np.float64)
    # Grid space (cell centres) to geographic.
    seg[:, [0, 2]] = grid.west + (seg[:, [0, 2]] + 0.5) * grid.dlon
    seg[:, [1, 3]] = grid.north - (seg[:, [1, 3]] + 0.5) * grid.dlat
    return seg
