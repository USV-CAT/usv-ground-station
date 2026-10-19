"""Drawing stays cheap when zoomed out: the cached map image and simplified tiles."""

import numpy as np
import pytest

QtWidgets = pytest.importorskip("PyQt5.QtWidgets")


def _render(canvas):
    from PyQt5 import QtGui
    image = QtGui.QImage(canvas.width(), canvas.height(), QtGui.QImage.Format_RGB32)
    painter = QtGui.QPainter(image)
    canvas.render(painter)
    painter.end()


def test_map_image_is_reused_stretched_while_zooming_and_redrawn_after(chart):
    from usvgs.core.mission import Mission
    from usvgs.ui.mapview.canvas import ChartCanvas
    from usvgs.ui.mapview.depth import ChartLayer
    from usvgs.ui.mapview.tiles import TiledMap

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])  # noqa: F841 (kept alive)
    canvas = ChartCanvas(TiledMap(), Mission())
    canvas.chart = ChartLayer(chart, 1.0)
    canvas.resize(800, 500)
    canvas.follow = False
    canvas.set_scale(20_000)
    canvas._zooming = False

    _render(canvas)
    built = canvas._static
    assert built is not None
    canvas.pan_pixels(10, 5)
    _render(canvas)
    assert canvas._static is built            # small pan: blitted, not redrawn

    canvas.zoom(-3)
    _render(canvas)
    assert canvas._static is built            # zooming: stretched
    canvas._zoom_finished()
    _render(canvas)
    assert canvas._static is not built
    assert canvas._static.scale == canvas.scale

    redrawn = canvas._static
    canvas.chart.set_shallow_alarm(3.0)       # new hazard colours for the isobaths
    _render(canvas)
    assert canvas._static is not redrawn


def test_coarse_isobaths_have_fewer_segments(chart):
    from usvgs.ui.mapview.depth import ChartLayer

    layer = ChartLayer(chart, 1.0)
    fine, coarse = layer.isobaths_at(1), layer.isobaths_at(4)
    assert len(coarse[0.0]) < len(fine[0.0]) / 2
    # The coarse coastline still runs where the fine one does.
    west, east = fine[0.0][:, 0].min(), fine[0.0][:, 0].max()
    assert abs(coarse[0.0][:, 0].min() - west) < 5 * chart.dlon
    assert abs(coarse[0.0][:, 0].max() - east) < 5 * chart.dlon


def test_simplify_merges_close_points_and_keeps_ends():
    from usvgs.ui.mapview.tiles import simplify

    # Two lines: a 100-point wiggle 1e-5 degrees apart, then a 2-point line.
    lat = np.concatenate([57.0 + np.arange(100) * 1e-5, [57.1, 57.2]])
    lon = np.concatenate([24.0 + np.arange(100) * 1e-5, [24.1, 24.2]])
    counts = np.array([100, 2])
    s_lat, s_lon, s_counts = simplify(lat, lon, counts, 2e-4)
    assert list(s_counts) == [s_counts[0], 2] and 2 <= s_counts[0] < 15
    assert s_lat[0] == lat[0] and s_lat[s_counts[0] - 1] == lat[99]
    assert list(s_lat[-2:]) == [57.1, 57.2]


def test_natural_water_images_and_wave_frames(chart):
    from usvgs.ui.mapview.depth import ChartLayer
    from usvgs.ui.mapview.water import FRAMES, TILE, WaterSurface, water_rgb

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])  # noqa: F841 (kept alive)
    layer = ChartLayer(chart, 1.0)
    water = WaterSurface()
    colour, ground = water.images(layer)
    assert colour.width() == chart.shape[1] and ground.height() == chart.shape[0]
    assert water.images(layer)[0] is colour          # cached
    layer.set_shallow_alarm(2.5)
    assert water.images(layer)[0] is not colour      # the hazard tint follows the alarm
    # Shallow water is lighter and greener than deep water.
    shallow, deep = water_rgb(np.array([0.5, 40.0]))
    assert shallow.sum() > deep.sum() * 1.8 and shallow[1] > deep[1]
    frames = water.frames
    assert len(frames) == FRAMES and frames[0].width() == TILE
    assert water.strength(10.0) > water.strength(0.0)
    assert water.strength(5.0, overhead=True) < water.strength(5.0)


@pytest.mark.parametrize("mode", ["north", "heading", "chase"])
@pytest.mark.parametrize("style", ["natural", "chart"])
def test_both_water_styles_render_in_every_view(chart, mode, style):
    from usvgs.core.mission import Mission
    from usvgs.ui.mapview.canvas import ChartCanvas
    from usvgs.ui.mapview.depth import ChartLayer
    from usvgs.ui.mapview.tiles import TiledMap

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])  # noqa: F841 (kept alive)
    canvas = ChartCanvas(TiledMap(), Mission())
    canvas.chart = ChartLayer(chart, 1.0)
    canvas.resize(640, 400)
    canvas.set_boat(57.05, 23.95, 300.0, 300.0, 0.5, 0.0)
    canvas.vectors = {"wind": (6.0, 250.0)}
    canvas.set_water_style(style)
    canvas.set_view_mode(mode)
    _render(canvas)
    _render(canvas)
    assert canvas.natural_water == (style == "natural")
    assert canvas._static.key[-1] == canvas.natural_water


def test_sun_follows_the_clock():
    import datetime

    from usvgs.ui.mapview.canvas import ChartCanvas

    morning = ChartCanvas.sun_azimuth(datetime.datetime(2026, 6, 1, 8, 0))
    noon = ChartCanvas.sun_azimuth(datetime.datetime(2026, 6, 1, 13, 0))
    evening = ChartCanvas.sun_azimuth(datetime.datetime(2026, 6, 1, 19, 0))
    assert 90 < morning < 130 and noon == 180 and 250 < evening < 280


def test_boat_is_carried_on_between_reports_and_glides_onto_new_fixes():
    import time

    from usvgs.core import geo
    from usvgs.core.mission import Mission
    from usvgs.ui.mapview.canvas import ChartCanvas
    from usvgs.ui.mapview.tiles import TiledMap

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])  # noqa: F841 (kept alive)
    canvas = ChartCanvas(TiledMap(), Mission())
    start = (57.05, 23.95)
    canvas.set_boat(*start, 90.0, 90.0, 2.0, 0.0, rot=10.0)   # east at 2 m/s, turning
    t0 = canvas._fix_time
    canvas.advance(t0 + 0.25)
    moved = geo.distance_m(start, canvas.boat[:2])
    assert 0.45 < moved < 0.55                       # 2 m/s for a quarter second
    assert abs(canvas.boat[2] - 92.5) < 0.01         # 10 °/s
    canvas.advance(t0 + 5.0)                         # no reports: it stops guessing
    assert geo.distance_m(start, canvas.boat[:2]) < 2.0 * 0.4 + 0.05

    # A new fix 3 m off: drawn where the boat was, then eased onto it.
    shown = canvas.boat
    fix = geo.destination(start, 0.0, 3.0)
    canvas.set_boat(fix.lat, fix.lon, 90.0, 90.0, 0.0, 0.0)
    assert geo.distance_m(shown[:2], canvas.boat[:2]) < 0.05
    canvas.advance(time.monotonic() + 1.0)
    assert geo.distance_m((fix.lat, fix.lon), canvas.boat[:2]) < 0.01


def test_frame_rate_setting_and_counter(chart):
    from usvgs import config
    from usvgs.ui.app import MainWindow
    from usvgs.ui.mapview.tiles import TiledMap

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])  # noqa: F841 (kept alive)
    settings = config.Settings()
    settings.control.input_backend = "none"
    window = MainWindow(settings, chart, TiledMap())
    try:
        assert window._frame_timer.interval() == 16
        window.set_max_fps(30)
        assert window._frame_timer.interval() == 33 and settings.ui.max_fps == 30
        window.set_show_fps(True)
        window.resize(1000, 700)
        _render(window.canvas)
        fps, ms = window.canvas.frame_stats()
        assert fps >= 1 and ms > 0
    finally:
        window.close()


def test_slick_image_and_survey_palette():
    from usvgs.ui.mapview.depth import survey_rgb, survey_rgb_array
    from usvgs.ui.mapview.slick import colour_film

    depths = np.linspace(-1, 12, 40)
    assert np.all(np.abs(survey_rgb_array(depths, 2.0, 9.0).astype(int)
                         - np.array([survey_rgb(d, 2.0, 9.0) for d in depths])) <= 1)
    sheen, brown, black = colour_film(np.array([0.3, 10.0, 80.0]))
    assert sheen[3] < brown[3] < black[3]            # thicker oil is more opaque
    assert black[:3].sum() < brown[:3].sum()         # and darker
