import math

import numpy as np
import pytest

from usvgs.core.bathymetry import ESTIMATED, DepthGrid, SoundingGrid, contour_segments
from usvgs.core.geo import LatLon
from usvgs.core.mission import Rectangle


def test_bundled_chart_knows_sea_from_land(chart):
    assert chart.depth_at(57.10, 23.95) > 10.0          # Gulf of Riga
    assert math.isnan(chart.depth_at(56.95, 24.12))     # Old Riga is not water
    assert chart.quality_at(57.02, 24.18) == ESTIMATED  # Ķīšezers is estimated


def test_npz_round_trip(tmp_path):
    depth = np.array([[1.0, 2.0], [np.nan, 4.5]], np.float32)
    grid = DepthGrid(depth, 57.0, 24.0, 0.001, 0.002, source="t")
    grid.save_npz(tmp_path / "g.npz")
    again = DepthGrid.load(tmp_path / "g.npz")
    assert np.isnan(again.depth[1, 0])
    assert again.depth[1, 1] == pytest.approx(4.5)
    assert again.dlon == pytest.approx(0.002)


def test_interpolation_is_bilinear_in_open_water():
    depth = np.array([[2.0, 4.0], [2.0, 4.0]], np.float32)
    grid = DepthGrid(depth, 1.0, 0.0, 0.5, 0.5)
    assert grid.depth_at(0.5, 0.5) == pytest.approx(3.0)


def test_ascii_grid_is_read_as_elevation(tmp_path):
    path = tmp_path / "g.asc"
    path.write_text("ncols 2\nnrows 2\nxllcorner 24.0\nyllcorner 57.0\ncellsize 0.01\n"
                    "NODATA_value -9999\n-3 -5\n2 -9999\n")
    grid = DepthGrid.load(path)
    assert grid.depth[0, 0] == pytest.approx(3.0)
    assert np.isnan(grid.depth[1, 0]) and np.isnan(grid.depth[1, 1])
    assert grid.north == pytest.approx(57.02)


def test_xyz_soundings_are_binned(tmp_path):
    path = tmp_path / "s.csv"
    path.write_text("lon,lat,depth\n24.0,57.0,3.0\n24.0001,57.0001,5.0\n24.01,57.01,9.0\n")
    grid = DepthGrid.load(path)
    assert np.nanmax(grid.depth) == pytest.approx(9.0)


def test_contours_follow_the_level():
    depth = np.tile(np.arange(10, dtype=np.float32), (5, 1))
    grid = DepthGrid(depth, 1.0, 0.0, 0.1, 0.1)
    segments = contour_segments(grid, 4.5)
    assert len(segments) == 4
    # Every crossing sits half-way between the columns of depth 4 and 5.
    expected_lon = grid.west + (4.5 + 0.5) * grid.dlon
    assert np.allclose(segments[:, [0, 2]], expected_lon)


def test_soundings_fill_cells_and_coverage():
    origin = LatLon(57.0, 24.0)
    grid = SoundingGrid(origin, cell_m=5.0)
    rect = Rectangle(origin, 20.0, 20.0, 90.0)
    assert grid.coverage(rect) == 0.0
    new = grid.add(57.0, 24.0, 3.2, swath_m=10.0)
    assert new > 1
    assert grid.depth_of(grid.cell_of(57.0, 24.0)) == pytest.approx(3.2)
    assert 0.0 < grid.coverage(rect) <= 1.0
    grid.add(57.0, 24.0, 3.6)
    assert grid.depth_of(grid.cell_of(57.0, 24.0)) == pytest.approx(3.4)


def test_soundings_export(tmp_path):
    grid = SoundingGrid(LatLon(57.0, 24.0))
    grid.add(57.0, 24.0, 2.0)
    assert grid.export_xyz(tmp_path / "x.csv") == 1
    reread = DepthGrid.load_xyz(tmp_path / "x.csv")
    assert np.nanmax(reread.depth) == pytest.approx(2.0)
