"""Colours and styling.

Dark, high-contrast and large: the Steam Deck's 7-inch screen is read in
daylight at arm's length, often with a thumb rather than a mouse.  Colours carry
meaning and mean the same everywhere — green is good and live, amber is
attention, red is act now.
"""

from __future__ import annotations

from PyQt5 import QtGui

BACKGROUND = "#0b1016"
SURFACE = "#111922"
SURFACE_RAISED = "#18232f"
BORDER = "#253445"
TEXT = "#e3ecf4"
TEXT_DIM = "#7f91a3"

ACCENT = "#36c6f4"      # cyan: the HUD
GOOD = "#5ee07a"        # green: live, armed, OK
CAUTION = "#ffb53d"
BAD = "#ff5a4e"
CRITICAL = "#ff2d55"
VALUE = "#ffd84a"       # telemetry numbers, as on the reference station

#: Chart.
LAND = "#1a1d16"
LAND_EDGE = "#4b5a3b"
WATER_DEEP = (8, 22, 44)
WATER_SHALLOW = (38, 130, 190)
WATER_HAZARD = (255, 120, 70)
CONTOUR = (120, 190, 235)
TRACK = "#ff9f3c"
ROUTE = "#ffffff"
ROUTE_ACTIVE = "#ffb53d"
WAYPOINT = "#36c6f4"
WAYPOINT_DONE = "#5d6f80"
AREA = "#5ee07a"
HOME = "#5ee07a"
BOAT = "#ffd84a"
TARGET = "#ff5ad8"

#: OSM vector layers over the chart, as (red, green, blue, alpha, width).
MAP_LAYERS = {
    "landuse": (70, 90, 60, 110, 1),
    "water": (90, 170, 230, 150, 1),
    "building": (110, 110, 120, 120, 1),
    "rail": (150, 120, 95, 140, 1),
    "road_minor": (130, 130, 135, 110, 1),
    "road_major": (220, 185, 100, 170, 2),
}
MAP_LAYER_ORDER = ("landuse", "water", "building", "rail", "road_minor", "road_major")


def qcolor(name: str, alpha: int | None = None) -> QtGui.QColor:
    colour = QtGui.QColor(name)
    if alpha is not None:
        colour.setAlpha(alpha)
    return colour


def severity_colour(level: int) -> str:
    if level >= 4:
        return CRITICAL
    if level == 3:
        return BAD
    if level == 2:
        return CAUTION
    if level == 1:
        return ACCENT
    if level < 0:
        return TEXT_DIM
    return TEXT


def battery_colour(percent: float | None) -> str:
    if percent is None:
        return TEXT_DIM
    if percent > 40:
        return GOOD
    if percent > 15:
        return CAUTION
    return BAD


def depth_rgb(depth: float, shallow_alarm: float, max_depth: float = 30.0) -> tuple[int, int, int]:
    """Chart colour for a depth: bright and shallow to dark and deep, hazards hot."""
    if depth < shallow_alarm:
        return WATER_HAZARD
    f = min(1.0, (depth / max_depth) ** 0.55)
    return tuple(int(s + (d - s) * f) for s, d in zip(WATER_SHALLOW, WATER_DEEP, strict=True))


STYLESHEET = f"""
QWidget {{
    background: {BACKGROUND};
    color: {TEXT};
    font-size: 13px;
}}
QToolTip {{ background: {SURFACE_RAISED}; color: {TEXT}; border: 1px solid {BORDER}; padding: 4px; }}

QPushButton, QToolButton {{
    background: {SURFACE_RAISED};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 7px 10px;
    min-height: 26px;
}}
QPushButton:hover, QToolButton:hover {{ border-color: {ACCENT}; }}
QPushButton:pressed, QToolButton:pressed {{ background: #0d151d; }}
QPushButton:checked, QToolButton:checked {{
    background: #12384a; border: 2px solid {ACCENT}; color: #ffffff; font-weight: 700;
}}
QPushButton:disabled {{ color: #4a5966; border-color: #1b2632; }}

QPushButton#mode {{ font-size: 14px; font-weight: 600; padding: 6px 8px; }}
QPushButton#mode:checked {{ background: #163d22; border: 2px solid {GOOD}; color: #eaffef; }}
QPushButton#stop {{ background: #4a1013; border: 2px solid {BAD}; color: #ffe6e3;
    font-size: 15px; font-weight: 800; }}
QPushButton#stop:checked {{ background: #8f1c1f; }}
QPushButton#arm {{ background: #3c2a0a; border: 2px solid {CAUTION}; font-weight: 800;
    font-size: 14px; }}
QPushButton#arm:checked {{ background: #16451f; border-color: {GOOD}; color: #eaffef; }}
QToolButton#tool {{ font-size: 11px; min-width: 56px; padding: 4px 2px; }}

QFrame#card {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 8px; }}
QLabel#heading {{ color: {ACCENT}; font-weight: 700; font-size: 12px; letter-spacing: 1px; }}
QLabel#dim {{ color: {TEXT_DIM}; }}

QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 6px; top: -1px; background: {SURFACE}; }}
QTabBar::tab {{
    background: {SURFACE}; color: {TEXT_DIM}; padding: 8px 12px; border: 1px solid {BORDER};
    border-bottom: none; border-top-left-radius: 6px; border-top-right-radius: 6px;
}}
QTabBar::tab:selected {{ background: {SURFACE_RAISED}; color: #ffffff; }}

QListWidget, QPlainTextEdit, QTableWidget {{
    background: #0a1118; border: 1px solid {BORDER}; border-radius: 6px;
}}
QListWidget::item:selected {{ background: #12384a; }}
QHeaderView::section {{ background: {SURFACE_RAISED}; border: 0; padding: 4px; color: {TEXT_DIM}; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background: #0a1118; border: 1px solid {BORDER}; border-radius: 5px; padding: 4px 6px;
    min-height: 24px; selection-background-color: {ACCENT};
}}
QComboBox::drop-down {{ border: 0; width: 20px; }}
QComboBox QAbstractItemView {{ background: {SURFACE_RAISED}; }}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {{ width: 22px; }}

QSlider::groove:horizontal {{ height: 6px; background: {BORDER}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: {ACCENT}; width: 20px; margin: -8px 0; border-radius: 10px; }}

QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 20px; height: 20px; }}
QCheckBox::indicator:unchecked {{ border: 1px solid {BORDER}; border-radius: 4px; background: #0a1118; }}
QCheckBox::indicator:checked {{ border: 1px solid {ACCENT}; border-radius: 4px; background: {ACCENT}; }}

QScrollBar:vertical {{ background: {BACKGROUND}; width: 12px; }}
QScrollBar::handle:vertical {{ background: {BORDER}; border-radius: 6px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
QScrollArea {{ border: 0; }}
"""
