"""Build an offline depth chart.

Fetches a piece of the EMODnet Bathymetry digital terrain model while there is
an internet connection, and writes it as a compact ``.npz`` the ground station
loads with no network::

    python -m usvgs.tools.bathy_build --bbox 23.40,56.90,24.60,57.45 --out riga.npz
    python -m usvgs.tools.bathy_build --tif downloaded.tif --out riga.npz

EMODnet covers European seas at 1/16 arc-minute (about 115 m).  Inland lakes
carry a flat placeholder rather than soundings, so this replaces those with an
*estimate* that deepens with distance from the shore, and flags those cells as
estimated so the interface can say so.  Measure a lake yourself with the
survey tools and export it if you need the real thing.
"""

from __future__ import annotations

import argparse
import sys
import urllib.request
from pathlib import Path

import numpy as np

from ..core.bathymetry import ESTIMATED, DepthGrid, read_geotiff

WCS = (
    "https://ows.emodnet-bathymetry.eu/wcs?service=wcs&version=1.0.0&request=getcoverage"
    "&coverage=emodnet:mean&crs=EPSG:4326&BBOX={w},{s},{e},{n}&format=image/tiff"
    "&interpolation=nearest&resx={res}&resy={res}"
)
#: EMODnet's native resolution, degrees.
NATIVE_RES = 1.0 / 960.0


def fetch(bbox: tuple[float, float, float, float], path: Path, res: float = NATIVE_RES) -> Path:
    w, s, e, n = bbox
    url = WCS.format(w=w, s=s, e=e, n=n, res=res)
    print(f"[*] fetching {url}", file=sys.stderr)
    with urllib.request.urlopen(url, timeout=180) as response:
        path.write_bytes(response.read())
    return path


def _erode_distance(water: np.ndarray, limit: int) -> np.ndarray:
    """Cells from each water cell to the nearest non-water cell, capped at ``limit``."""
    distance = np.zeros(water.shape, np.float32)
    current = water.copy()
    for step in range(1, limit + 1):
        distance[current] = step
        shrunk = current.copy()
        shrunk[1:] &= current[:-1]
        shrunk[:-1] &= current[1:]
        shrunk[:, 1:] &= current[:, :-1]
        shrunk[:, :-1] &= current[:, 1:]
        if not shrunk.any():
            break
        current = shrunk
    return distance


def estimate_lakes(elevation: np.ndarray, cell_m: float, max_depth_m: float = 4.5,
                   slope_per_m: float = 0.012) -> tuple[np.ndarray, np.ndarray]:
    """Replace EMODnet's flat lake placeholder with a shore-distance estimate.

    Returns ``(depth, quality)`` with depth positive down and NaN on land.
    """
    depth = np.where(elevation < 0.0, -elevation, np.nan).astype(np.float32)
    quality = np.zeros(elevation.shape, np.uint8)
    # The placeholder is a run of identical shallow values.  Real soundings
    # vary from cell to cell, so a flat patch at exactly that value is a lake.
    placeholder = np.isclose(elevation, -1.0, atol=1e-4)
    if not placeholder.any():
        return depth, quality
    distance = _erode_distance(placeholder, limit=int(max_depth_m / slope_per_m / cell_m) + 2)
    estimate = np.minimum(max_depth_m, 0.4 + distance * cell_m * slope_per_m)
    depth[placeholder] = estimate[placeholder]
    quality[placeholder] = ESTIMATED
    return depth, quality


def build(tif: Path, out: Path) -> DepthGrid:
    elevation, north, west, dlat, dlon = read_geotiff(tif)
    cell_m = dlat * 111_320.0
    depth, quality = estimate_lakes(elevation, cell_m)
    grid = DepthGrid(depth, north, west, dlat, dlon, quality,
                     source="EMODnet Bathymetry DTM (lakes estimated)")
    grid.save_npz(out)
    water = np.isfinite(depth)
    print(
        f"[*] wrote {out}: {grid.shape[1]}×{grid.shape[0]} cells, "
        f"{water.mean():.0%} water, deepest {np.nanmax(depth):.1f} m, "
        f"{(quality == ESTIMATED).sum()} estimated cells",
        file=sys.stderr,
    )
    return grid


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bbox", help="west,south,east,north in degrees")
    source.add_argument("--tif", type=Path, help="an EMODnet GeoTIFF already downloaded")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    tif = args.tif
    if args.bbox:
        bbox = tuple(float(v) for v in args.bbox.split(","))
        if len(bbox) != 4:
            parser.error("--bbox needs four numbers")
        tif = fetch(bbox, args.out.with_suffix(".tif"))
    build(tif, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
