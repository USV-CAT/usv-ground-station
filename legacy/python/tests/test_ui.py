"""The window builds, plans a rectangle and drives, with no display."""

import time

import pytest

QtWidgets = pytest.importorskip("PyQt5.QtWidgets")


def test_window_smoke(chart):
    from usvgs import config
    from usvgs.core import geo
    from usvgs.core.mission import Rectangle
    from usvgs.ui.app import MainWindow
    from usvgs.ui.mapview.tiles import TiledMap
    from usvgs.vehicle.modes import Mode

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    settings = config.Settings()
    settings.control.input_backend = "none"
    window = MainWindow(settings, chart, TiledMap())
    window.resize(1280, 800)
    window.show()
    try:
        end = time.time() + 1.0
        while time.time() < end:
            app.processEvents()
            time.sleep(0.01)
        assert window.vehicle.nav().telemetry is not None
        boat = window.boat_position()
        with window.vehicle.lock:
            window.vehicle.mission.area = Rectangle(geo.destination(boat, 0, 100), 60, 30, 90)
        window.regenerate_route()
        assert len(window.vehicle.mission.waypoints) > 4
        window.vehicle.arm()
        window.request_mode(Mode.AUTO)
        assert window.vehicle.mode is Mode.AUTO
        window._button("B")
        assert window.vehicle.mode is Mode.STOP
        image = window.grab()
        assert image.width() == 1280

        # Every page and every view paints.
        nav = window.vehicle.nav()
        for title in window.pages:
            window.show_page(title)
            window._refresh()
            window._refresh_slow()
        window.depth_page.refresh(nav)
        window.power_page.refresh(nav)
        window.oil_page.refresh(nav)
        window.oil_page.plan_sweep()
        assert window.vehicle.mission.drift_ms is not None
        for _ in range(3):
            window.cycle_view()
            window._refresh()
            window.grab()
        window.toggle_telemetry()
        window.toggle_drawer()
        assert not window.telemetry_panel.isVisible() and not window.drawer.isVisible()
        window._button("L3")
        assert window.vehicle.nav().skimmer_commanded or window.vehicle._skimmer
    finally:
        window.close()
