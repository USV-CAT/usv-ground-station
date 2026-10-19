//! The chart.
//!
//! Pan, zoom, the boat and its track, home, the route and the survey
//! rectangle, the oil slick and the vectors acting on the boat.  Four tools
//! decide what a tap on the water does:
//!
//! * **pan** — nothing but moving the chart;
//! * **points** — add a waypoint; drag one to move it; right-click (or select
//!   and press Delete) to remove it;
//! * **rect** — drag out a survey area, then drag its corners to resize, its
//!   middle to move it and the round handle to rotate it;
//! * **go to** — send the boat to the tapped point (Point Nav).
//!
//! Three views: **north up**; **heading up**, where the chart turns so the
//! bow points up and there is more water ahead than astern; and **chase**, a
//! third-person camera behind the boat with the heading-up chart laid out in
//! perspective under a sky.
//!
//! Two looks for the water: **natural** (the colour of water over that
//! depth, moving waves, surf on the beach, and in the chase view the sky
//! reflected towards the horizon and the sun's glitter) and **chart** (depth
//! shading).  Isobaths, hazards and soundings show in both.

pub mod draw;
pub mod images;
pub mod layers;
pub mod projection;
pub mod satellite;
pub mod static_layer;
pub mod tiles;

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;

use egui::epaint::TextShape;
use egui::{Align2, Color32, Context, CursorIcon, Painter, Pos2, Rect, Sense, Shape, Stroke, TextureHandle, TextureOptions, Ui, Vec2};

use crate::config::WaterStyle;
use crate::core::bathymetry::DepthGrid;
use crate::core::clock;
use crate::core::geo::{self, BBox, LatLon};
use crate::core::mission::{Mission, Rectangle};
use crate::core::rng::Rng;
use crate::sim::oil::SpillSnapshot;
use crate::ui::theme;
use crate::vehicle::Vehicle;

use layers::{ChartPicture, Sea, SlickLayer, SurveyLayer, WaveLayer};
use projection::{Camera, Chase, CHASE_AHEAD, CHASE_RES};
use static_layer::{StaticKey, StaticRenderer, StaticRequest};

/// Pixels per degree of latitude: about 30 km across.
pub const MIN_SCALE: f64 = 2_000.0;
/// The chase camera stops further in (about 5 km across): its ground plane
/// reaches far past the screen.
pub const CHASE_MIN_SCALE: f64 = 12_000.0;
/// About half a centimetre a pixel.
pub const MAX_SCALE: f64 = 20_000_000.0;
pub const ZOOM_STEP: f64 = 1.3;
/// Generous: fingers, not mouse pointers.
pub const HIT_PX: f32 = 22.0;
/// The side tabs on the chart's edges are this wide; legends keep clear of them.
pub const SIDE_TAB_W: f32 = 30.0;
pub const TRACK_DRAW_LIMIT: usize = 2500;

/// Between position reports the boat is carried on by its motion for at
/// most this long; a correction is blended out over about this time; and the
/// heading-up views turn with this time constant.
pub const PREDICT_MAX_S: f64 = 0.4;
pub const GLIDE_S: f64 = 0.12;
pub const TURN_EASE_S: f64 = 0.17;

/// Spare round the view in the cached map image, as a fraction of its longer side.
pub const STATIC_MARGIN: f64 = 0.25;
/// A zoom counts as finished once it has been still this long.
pub const ZOOM_SETTLE_S: f64 = 0.16;

/// Daylight sky over natural water, top to horizon; the horizon colour is
/// also what the water reflects far off.
pub const SKY_TOP: Color32 = Color32::from_rgb(46, 98, 158);
pub const SKY_HORIZON: Color32 = Color32::from_rgb(172, 204, 226);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Pan,
    Points,
    Rect,
    Goto,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewMode {
    North,
    Heading,
    Chase,
}

impl ViewMode {
    pub fn label(self) -> &'static str {
        match self {
            ViewMode::North => "N↑",
            ViewMode::Heading => "H↑",
            ViewMode::Chase => "3D",
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            ViewMode::North => "North up",
            ViewMode::Heading => "Heading up",
            ViewMode::Chase => "3D chase camera",
        }
    }

    pub fn next(self) -> ViewMode {
        match self {
            ViewMode::North => ViewMode::Heading,
            ViewMode::Heading => ViewMode::Chase,
            ViewMode::Chase => ViewMode::North,
        }
    }
}

/// What a frame of the chart did that the window should hear about.
#[derive(Clone, Debug, PartialEq)]
pub enum ChartEvent {
    MissionEdited,
    AreaEdited,
    Goto(LatLon),
    Cursor(LatLon),
    Selection(Option<usize>),
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoatVectors {
    /// Mean throttle and differential, fractions.
    pub thrust: f64,
    pub turn: f64,
    /// (speed, from) and (drift, set).
    pub wind: Option<(f64, f64)>,
    pub current: Option<(f64, f64)>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum RectDrag {
    Corner(usize),
    Move,
    Rotate,
}

struct Press {
    at: Pos2,
    centre: LatLon,
    moved: bool,
    drag_wp: Option<usize>,
    rect_anchor: Option<LatLon>,
    rect_drag: Option<(RectDrag, Rectangle, LatLon)>,
}

struct StaticTexture {
    request: StaticRequest,
    texture: TextureHandle,
}

pub struct ChartView {
    pub centre: LatLon,
    pub scale: f64,
    pub view_mode: ViewMode,
    pub rotation: f64,
    pub follow: bool,
    pub tool: Tool,
    pub selected: Option<usize>,
    pub rect_angle_deg: f64,

    pub show_chart: bool,
    pub show_isobaths: bool,
    pub show_survey: bool,
    pub show_osm: bool,
    pub show_track: bool,
    pub show_leg_lengths: bool,
    pub show_vectors: bool,
    pub show_oil: bool,
    pub water_style: WaterStyle,
    pub show_fps: bool,
    /// Simulated time runs this much faster than the clock.
    pub time_scale: f64,
    pub shallow_alarm_m: f64,

    pub boat_length_m: f64,
    pub boat_beam_m: f64,
    pub geofence_m: Option<f64>,
    pub station: Option<LatLon>,
    pub home: Option<LatLon>,
    pub target: Option<LatLon>,
    pub target_label: String,
    pub active_index: Option<usize>,
    pub vectors: BoatVectors,
    /// What moves the water, for the waves.
    pub sea: Sea,
    pub skimmer_deployed: bool,
    pub skimmer_swath_m: f64,
    pub oil: Option<SpillSnapshot>,

    /// Where the boat is drawn: lat, lon, heading.
    pub boat: Option<(f64, f64, f64)>,
    boat_cog: Option<(f64, f64)>,
    boat_roll: f64,
    fix: Option<(f64, f64, f64)>,
    fix_time: f64,
    fix_motion: (Option<f64>, f64, f64),
    fix_error: (f64, f64, f64),
    advanced_at: f64,

    press: Option<Press>,
    zooming_until: f64,
    last_cursor: Option<Pos2>,
    initial_zoom_m: Option<f64>,

    chart: Option<(Arc<DepthGrid>, u64)>,
    picture: ChartPicture,
    renderer: StaticRenderer,
    static_tex: Option<StaticTexture>,
    pending: Option<StaticRequest>,
    waves: Option<WaveLayer>,
    satellite: Option<satellite::Satellite>,
    /// Satellite tile source and its credit.
    pub satellite_url: String,
    pub satellite_attribution: String,
    survey: SurveyLayer,
    slick: SlickLayer,

    frame_times: VecDeque<f64>,
    pub paint_ms: f64,
    rect: Rect,
}

impl ChartView {
    pub fn new(ctx: &Context, map_index: Option<PathBuf>) -> ChartView {
        ChartView {
            centre: LatLon::new(57.048, 23.99),
            scale: 180_000.0,
            view_mode: ViewMode::North,
            rotation: 0.0,
            follow: true,
            tool: Tool::Pan,
            selected: None,
            rect_angle_deg: 90.0,
            show_chart: true,
            show_isobaths: true,
            show_survey: true,
            show_osm: true,
            show_track: true,
            show_leg_lengths: true,
            show_vectors: true,
            show_oil: true,
            water_style: WaterStyle::Natural,
            show_fps: false,
            time_scale: 1.0,
            shallow_alarm_m: 1.2,
            boat_length_m: 1.6,
            boat_beam_m: 0.9,
            geofence_m: None,
            station: None,
            home: None,
            target: None,
            target_label: String::new(),
            active_index: None,
            vectors: BoatVectors::default(),
            sea: Sea::default(),
            skimmer_deployed: false,
            skimmer_swath_m: 2.4,
            oil: None,
            boat: None,
            boat_cog: None,
            boat_roll: 0.0,
            fix: None,
            fix_time: 0.0,
            fix_motion: (None, 0.0, 0.0),
            fix_error: (0.0, 0.0, 0.0),
            advanced_at: clock::monotonic(),
            press: None,
            zooming_until: 0.0,
            last_cursor: None,
            initial_zoom_m: Some(600.0),
            chart: None,
            picture: ChartPicture::default(),
            renderer: StaticRenderer::new(map_index, ctx.clone()),
            static_tex: None,
            pending: None,
            waves: None,
            satellite: None,
            satellite_url: crate::config::ESRI_IMAGERY.into(),
            satellite_attribution: String::new(),
            survey: SurveyLayer::new(),
            slick: SlickLayer::default(),
            frame_times: VecDeque::new(),
            paint_ms: 0.0,
            rect: Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0)),
        }
    }

    pub fn set_chart(&mut self, chart: Option<Arc<DepthGrid>>) {
        let id = self.chart.as_ref().map(|c| c.1 + 1).unwrap_or(1);
        self.chart = chart.map(|c| (c, id));
        self.static_tex = None;
        self.pending = None;
    }

    pub fn natural_water(&self) -> bool {
        self.water_style == WaterStyle::Natural
    }

    /// The satellite layer, started on first use.
    pub fn satellite_mut(&mut self, ctx: &Context) -> &mut satellite::Satellite {
        let url = self.satellite_url.clone();
        self.satellite.get_or_insert_with(|| satellite::Satellite::new(ctx, &url))
    }

    /// South, west, north, east of what the chart shows now.
    pub fn view_bounds(&self) -> (f64, f64, f64, f64) {
        self.camera(self.rect).bounds()
    }

    // ------------------------------------------------------------------
    // The view
    // ------------------------------------------------------------------

    fn lon_scale(&self) -> f64 {
        self.centre.lat.to_radians().cos().max(0.05)
    }

    pub fn metres_per_pixel(&self) -> f64 {
        geo::metres_per_degree(self.centre.lat).0 / self.scale
    }

    /// How much water is across the shorter side of the view.
    pub fn view_width_m(&self) -> f64 {
        self.rect.width().min(self.rect.height()) as f64 * self.metres_per_pixel()
    }

    pub fn zooming(&self) -> bool {
        clock::monotonic() < self.zooming_until
    }

    /// The camera for the current view, in a screen rectangle.
    pub fn camera(&self, rect: Rect) -> Camera {
        if self.view_mode == ViewMode::Chase {
            let centre = self.chase_centre(rect).unwrap_or(self.centre);
            if let Some(cam) = Camera::chase(rect, centre, self.scale, self.rotation) {
                return cam;
            }
        }
        let rotation = if self.view_mode == ViewMode::North { 0.0 } else { self.rotation };
        Camera::flat(rect, self.centre, self.scale, rotation)
    }

    /// The middle of the chase camera's ground image: ahead of the boat.
    fn chase_centre(&self, rect: Rect) -> Option<LatLon> {
        let (lat, lon, _) = self.boat?;
        let (_, gh) = Chase::ground_size(rect);
        let mpp = geo::metres_per_degree(lat).0 / (self.scale * CHASE_RES);
        Some(geo::destination(LatLon::new(lat, lon), self.rotation, gh * CHASE_AHEAD * mpp))
    }

    pub fn set_view_mode(&mut self, mode: ViewMode) {
        self.view_mode = mode;
        if mode != ViewMode::North {
            self.follow = true;
        } else {
            self.rotation = 0.0;
        }
        if mode == ViewMode::Chase && self.scale < CHASE_MIN_SCALE {
            self.scale = CHASE_MIN_SCALE;
        }
        self.update_follow();
    }

    pub fn cycle_view(&mut self) -> ViewMode {
        self.set_view_mode(self.view_mode.next());
        self.view_mode
    }

    pub fn set_scale(&mut self, scale: f64, focus: Option<Pos2>) {
        let lowest = if self.view_mode == ViewMode::Chase { CHASE_MIN_SCALE } else { MIN_SCALE };
        let scale = scale.clamp(lowest, MAX_SCALE);
        match focus {
            Some(f) if !self.follow && self.view_mode != ViewMode::Chase => {
                let before = self.camera(self.rect).unproject(f);
                self.scale = scale;
                let after = self.camera(self.rect).unproject(f);
                if let (Some(b), Some(a)) = (before, after) {
                    self.centre.lat += b.lat - a.lat;
                    self.centre.lon += b.lon - a.lon;
                }
            }
            _ => {
                self.scale = scale;
                self.update_follow();
            }
        }
        self.zooming_until = clock::monotonic() + ZOOM_SETTLE_S;
    }

    pub fn zoom(&mut self, steps: f64, focus: Option<Pos2>) {
        self.set_scale(self.scale * ZOOM_STEP.powf(steps), focus);
    }

    /// Show `metres` of water across the shorter side of the view.
    pub fn zoom_to_metres(&mut self, metres: f64) {
        let lat_m = geo::metres_per_degree(self.centre.lat).0;
        self.set_scale(self.rect.width().min(self.rect.height()) as f64 / (metres / lat_m), None);
    }

    pub fn centre_on(&mut self, p: LatLon) {
        self.centre = p;
    }

    pub fn pan_pixels(&mut self, dx: f64, dy: f64) {
        if self.view_mode == ViewMode::Chase {
            return;
        }
        self.follow = false;
        let rotation = if self.view_mode == ViewMode::North { 0.0 } else { self.rotation };
        let (s, c) = rotation.to_radians().sin_cos();
        let (mx, my) = (dx * c - dy * s, dx * s + dy * c);
        self.centre.lat += my / self.scale;
        self.centre.lon -= mx / (self.scale * self.lon_scale());
    }

    pub fn frame(&mut self, points: &[LatLon]) {
        let Some(bbox) = BBox::around(points, 40.0) else { return };
        if self.view_mode != ViewMode::North {
            self.set_view_mode(ViewMode::North);
        }
        self.follow = false;
        self.centre = bbox.centre();
        let span_lat = (bbox.max_lat - bbox.min_lat).max(1e-6);
        let span_lon = (bbox.max_lon - bbox.min_lon).max(1e-6) * self.lon_scale();
        let scale = (self.rect.height() as f64 / span_lat).min(self.rect.width() as f64 / span_lon) * 0.9;
        self.set_scale(scale, None);
    }

    // ------------------------------------------------------------------
    // The boat between reports
    // ------------------------------------------------------------------

    /// A position report.  Between reports the boat is drawn where its speed,
    /// course and rate of turn say it has got to ([`Self::advance`]).
    #[allow(clippy::too_many_arguments)]
    pub fn set_boat(&mut self, lat: f64, lon: f64, heading: f64, cog: Option<f64>, sog: Option<f64>, roll: f64, rot: Option<f64>, now: f64) {
        let fix = (lat, lon, heading);
        if self.fix != Some(fix) {
            let shown = self.boat;
            if let (Some(shown), Some(_)) = (shown, self.fix) {
                // Glide from where the boat was drawn rather than jump.
                self.fix_time_shift(fix, cog, sog, rot, now, Some(shown));
            } else {
                self.fix_time_shift(fix, cog, sog, rot, now, None);
            }
        }
        self.boat_cog = cog.map(|c| (c, sog.unwrap_or(0.0)));
        self.boat_roll = roll;
        self.advance(now);
    }

    fn fix_time_shift(&mut self, fix: (f64, f64, f64), cog: Option<f64>, sog: Option<f64>, rot: Option<f64>, now: f64, shown: Option<(f64, f64, f64)>) {
        self.fix = Some(fix);
        self.fix_time = now;
        self.fix_motion = (cog, sog.unwrap_or(0.0), rot.unwrap_or(0.0));
        self.fix_error = match shown {
            None => (0.0, 0.0, 0.0),
            Some(s) => {
                let (lat1, lon1, hdg1) = self.predict(now);
                (s.0 - lat1, s.1 - lon1, geo::angle_difference(s.2, hdg1))
            }
        };
    }

    fn predict(&self, now: f64) -> (f64, f64, f64) {
        let Some((mut lat, mut lon, heading)) = self.fix else { return (0.0, 0.0, 0.0) };
        let (cog, sog, rot) = self.fix_motion;
        let dt = (now - self.fix_time).min(PREDICT_MAX_S) * self.time_scale;
        if let Some(cog) = cog {
            if sog > 0.02 {
                let (m_lat, m_lon) = geo::metres_per_degree(lat);
                let run = sog * dt;
                lat += run * cog.to_radians().cos() / m_lat;
                lon += run * cog.to_radians().sin() / m_lon;
            }
        }
        (lat, lon, (heading + rot * dt).rem_euclid(360.0))
    }

    /// Move the drawn boat and the camera on to `now`; call every frame.
    pub fn advance(&mut self, now: f64) {
        if self.fix.is_none() {
            return;
        }
        let (lat, lon, heading) = self.predict(now);
        let fade = (-(now - self.fix_time) / GLIDE_S).exp();
        let (e_lat, e_lon, e_hdg) = self.fix_error;
        let boat = (lat + e_lat * fade, lon + e_lon * fade, (heading + e_hdg * fade).rem_euclid(360.0));
        self.boat = Some(boat);
        let step = (now - self.advanced_at).clamp(0.0, 0.25);
        self.advanced_at = now;
        if self.view_mode != ViewMode::North {
            // Ease the turn rather than snapping, so the picture does not
            // shake with compass noise; the same at any frame rate.
            let k = 1.0 - (-step / TURN_EASE_S).exp();
            self.rotation = (self.rotation + geo::angle_difference(boat.2, self.rotation) * k).rem_euclid(360.0);
        }
        self.update_follow();
    }

    fn update_follow(&mut self) {
        if !self.follow {
            return;
        }
        let Some((lat, lon, _)) = self.boat else { return };
        match self.view_mode {
            ViewMode::North | ViewMode::Chase => self.centre = LatLon::new(lat, lon),
            ViewMode::Heading => {
                // Look ahead: the boat sits low in the view, the water ahead fills it.
                let ahead_px = self.rect.height() as f64 * 0.25;
                self.centre = geo::destination(LatLon::new(lat, lon), self.rotation, ahead_px * self.metres_per_pixel());
            }
        }
    }

    /// Frames drawn in the last second.
    pub fn fps(&mut self) -> f64 {
        let now = clock::monotonic();
        while self.frame_times.front().is_some_and(|t| now - t > 1.0) {
            self.frame_times.pop_front();
        }
        self.frame_times.len() as f64
    }

    // ------------------------------------------------------------------
    // The frame
    // ------------------------------------------------------------------

    pub fn ui(&mut self, ui: &mut Ui, rect: Rect, vehicle: &Vehicle) -> Vec<ChartEvent> {
        let started = std::time::Instant::now();
        let now = clock::monotonic();
        let first = self.rect.size() != rect.size();
        self.rect = rect;
        if first {
            if let Some(m) = self.initial_zoom_m.take() {
                self.zoom_to_metres(m);
            }
        }
        self.advance(now);
        let response = ui.interact(rect, ui.id().with("chart"), Sense::click_and_drag());
        let mut events = Vec::new();
        self.handle_input(ui, rect, &response, vehicle, &mut events);
        if matches!(self.tool, Tool::Points | Tool::Rect | Tool::Goto) && response.contains_pointer() {
            ui.ctx().set_cursor_icon(CursorIcon::Crosshair);
        }

        let cam = self.camera(rect);
        let painter = ui.painter_at(rect);
        self.paint(ui.ctx(), &painter, &cam, vehicle, now);
        self.frame_times.push_back(now);
        self.paint_ms += (started.elapsed().as_secs_f64() * 1000.0 - self.paint_ms) * 0.1;
        if self.zooming() {
            ui.ctx().request_repaint_after(std::time::Duration::from_secs_f64(ZOOM_SETTLE_S));
        }
        events
    }

    // ------------------------------------------------------------------
    // Interaction
    // ------------------------------------------------------------------

    fn waypoint_at(&self, cam: &Camera, mission: &Mission, pos: Pos2) -> Option<usize> {
        if cam.is_chase() {
            return None;
        }
        let mut best = None;
        let mut best_d = HIT_PX * HIT_PX;
        for (i, w) in mission.waypoints.iter().enumerate() {
            if let Some(p) = cam.project(w.position()) {
                let d = (p - pos).length_sq();
                if d <= best_d {
                    best = Some(i);
                    best_d = d;
                }
            }
        }
        best
    }

    fn rotate_handle(cam: &Camera, area: &Rectangle) -> Option<Pos2> {
        let c = area.corners();
        let mid = LatLon::new((c[2].lat + c[3].lat) / 2.0, (c[2].lon + c[3].lon) / 2.0);
        let m = cam.project(mid)?;
        let centre = cam.project(area.centre)?;
        let d = m - centre;
        let len = d.length().max(1.0);
        Some(m + d / len * 36.0)
    }

    fn rect_hit(&self, cam: &Camera, area: &Rectangle, pos: Pos2) -> Option<RectDrag> {
        if cam.is_chase() {
            return None;
        }
        let near = |p: Pos2| (p.x - pos.x).abs() + (p.y - pos.y).abs() < HIT_PX * 1.4;
        if Self::rotate_handle(cam, area).is_some_and(near) {
            return Some(RectDrag::Rotate);
        }
        for (i, corner) in area.corners().iter().enumerate() {
            if cam.project(*corner).is_some_and(near) {
                return Some(RectDrag::Corner(i));
            }
        }
        if cam.project(area.centre).is_some_and(near) || cam.unproject(pos).is_some_and(|g| area.contains(g)) {
            return Some(RectDrag::Move);
        }
        None
    }

    fn handle_input(&mut self, ui: &Ui, rect: Rect, response: &egui::Response, vehicle: &Vehicle, events: &mut Vec<ChartEvent>) {
        let (pressed, released, secondary, down, pos, hover, scroll, zoom, touch) = ui.input(|i| {
            (
                i.pointer.primary_pressed(),
                i.pointer.primary_released(),
                i.pointer.secondary_pressed(),
                i.pointer.primary_down(),
                i.pointer.interact_pos(),
                i.pointer.hover_pos(),
                i.smooth_scroll_delta.y,
                i.zoom_delta(),
                i.multi_touch(),
            )
        });
        let over = response.contains_pointer();

        // Two fingers: pinch to zoom, and pan as they move together.
        if let Some(t) = touch.filter(|t| t.num_touches >= 2) {
            self.press = None;
            if (t.zoom_delta - 1.0).abs() > 1e-4 {
                self.set_scale(self.scale * t.zoom_delta as f64, Some(t.center_pos));
            }
            if !self.follow && t.translation_delta.length() > 0.0 {
                self.pan_pixels(t.translation_delta.x as f64, t.translation_delta.y as f64);
            }
        } else if over && (zoom - 1.0).abs() > 1e-4 {
            self.set_scale(self.scale * zoom as f64, hover);
        } else if over && scroll.abs() > 0.1 {
            self.zoom((scroll / 60.0) as f64, hover);
        }

        let cam = self.camera(rect);
        if let Some(h) = hover.filter(|_| over) {
            if self.last_cursor != Some(h) {
                self.last_cursor = Some(h);
                if let Some(g) = cam.unproject(h) {
                    events.push(ChartEvent::Cursor(g));
                }
            }
        }

        if secondary && over {
            if let Some(p) = hover {
                let mut v = vehicle.lock();
                if let Some(index) = self.waypoint_at(&cam, &v.mission, p) {
                    if matches!(self.tool, Tool::Points | Tool::Pan) {
                        v.mission.remove(index);
                        self.selected = None;
                        events.push(ChartEvent::MissionEdited);
                    }
                }
            }
        }

        if pressed && over {
            if let Some(p) = pos {
                let mut press = Press { at: p, centre: self.centre, moved: false, drag_wp: None, rect_anchor: None, rect_drag: None };
                if let Some(here) = cam.unproject(p) {
                    let v = vehicle.lock();
                    match self.tool {
                        Tool::Points => {
                            if let Some(i) = self.waypoint_at(&cam, &v.mission, p) {
                                self.selected = Some(i);
                                press.drag_wp = Some(i);
                                events.push(ChartEvent::Selection(Some(i)));
                            }
                        }
                        Tool::Rect if !cam.is_chase() => {
                            match v.mission.area.as_ref().and_then(|a| self.rect_hit(&cam, a, p).map(|h| (h, *a))) {
                                Some((hit, area)) => press.rect_drag = Some((hit, area, here)),
                                None => press.rect_anchor = Some(here),
                            }
                        }
                        Tool::Pan => {
                            if let Some(i) = self.waypoint_at(&cam, &v.mission, p) {
                                self.selected = Some(i);
                                events.push(ChartEvent::Selection(Some(i)));
                            }
                        }
                        _ => {}
                    }
                }
                self.press = Some(press);
            }
        }

        if down {
            if let (Some(p), Some(press)) = (pos, self.press.as_mut()) {
                if (p.x - press.at.x).abs() + (p.y - press.at.y).abs() > 6.0 {
                    press.moved = true;
                }
            }
            let moved = self.press.as_ref().is_some_and(|pr| pr.moved);
            if let (true, Some(p)) = (moved, pos) {
                if let Some(here) = cam.unproject(p) {
                    self.drag(here, p, vehicle, events, &cam);
                }
            }
        }

        if released {
            let Some(press) = self.press.take() else { return };
            let tapped = !press.moved;
            if press.drag_wp.is_some() && press.moved {
                events.push(ChartEvent::MissionEdited);
            }
            if press.rect_anchor.is_some() || press.rect_drag.is_some() {
                if !tapped {
                    events.push(ChartEvent::AreaEdited);
                }
                return;
            }
            if !tapped {
                return;
            }
            let Some(here) = cam.unproject(press.at) else { return };
            match self.tool {
                Tool::Points => {
                    let mut v = vehicle.lock();
                    if self.waypoint_at(&cam, &v.mission, press.at).is_none() {
                        v.mission.add(here.lat, here.lon);
                        v.mission.area = None;
                        v.mission.drift_ms = None;
                        self.selected = Some(v.mission.waypoints.len() - 1);
                        events.push(ChartEvent::Selection(self.selected));
                        events.push(ChartEvent::MissionEdited);
                    }
                }
                Tool::Goto => events.push(ChartEvent::Goto(here)),
                _ => {}
            }
        }
    }

    fn drag(&mut self, here: LatLon, pos: Pos2, vehicle: &Vehicle, events: &mut Vec<ChartEvent>, cam: &Camera) {
        let press = self.press.as_ref().unwrap();
        if let Some(i) = press.drag_wp {
            let mut v = vehicle.lock();
            if let Some(w) = v.mission.waypoints.get_mut(i) {
                w.lat = here.lat;
                w.lon = here.lon;
            }
            return;
        }
        if let Some(anchor) = press.rect_anchor {
            vehicle.lock().mission.area = Some(Rectangle::from_corners(anchor, here, self.rect_angle_deg));
            events.push(ChartEvent::AreaEdited);
            return;
        }
        if let Some((kind, start, press_geo)) = press.rect_drag {
            let area = match kind {
                RectDrag::Move => {
                    let offset = geo::to_metres(press_geo, here);
                    Rectangle::new(geo::to_lat_lon(start.centre, offset), start.width_m, start.height_m, start.angle_deg)
                }
                RectDrag::Rotate => {
                    // The handle is off the far edge, at +90° from the width axis.
                    let angle = (geo::bearing_deg(start.centre, here) + 90.0).rem_euclid(360.0);
                    self.rect_angle_deg = angle;
                    Rectangle::new(start.centre, start.width_m, start.height_m, angle)
                }
                RectDrag::Corner(i) => {
                    let opposite = start.corners()[(i + 2) % 4];
                    let area = Rectangle::from_corners(opposite, here, start.angle_deg);
                    if area.width_m < 4.0 || area.height_m < 4.0 {
                        return;
                    }
                    area
                }
            };
            vehicle.lock().mission.area = Some(area);
            events.push(ChartEvent::AreaEdited);
            return;
        }
        if cam.is_chase() {
            return;
        }
        // Otherwise: pan, the press point staying under the finger.
        let (at, centre) = (press.at, press.centre);
        self.centre = centre;
        self.pan_pixels((pos.x - at.x) as f64, (pos.y - at.y) as f64);
    }

    pub fn delete_selected(&mut self, vehicle: &Vehicle) -> bool {
        let Some(i) = self.selected else { return false };
        let mut v = vehicle.lock();
        if i >= v.mission.waypoints.len() {
            return false;
        }
        v.mission.remove(i);
        let n = v.mission.waypoints.len();
        self.selected = if n == 0 { None } else { Some(i.min(n - 1)) };
        true
    }

    // ------------------------------------------------------------------
    // Painting
    // ------------------------------------------------------------------

    fn paint(&mut self, ctx: &Context, painter: &Painter, cam: &Camera, vehicle: &Vehicle, now: f64) {
        let rect = cam.rect;
        let chase = cam.chase;
        let natural = self.natural_water();
        painter.rect_filled(rect, 0.0, theme::LAND);
        let ground = match &chase {
            Some(c) => {
                let horizon = c.horizon_y as f32;
                let far = if natural { SKY_HORIZON } else { Color32::from_rgb(70, 110, 150) };
                let top = if natural { SKY_TOP } else { Color32::from_rgb(14, 30, 58) };
                draw::vertical_gradient(painter, Rect::from_min_max(rect.min, Pos2::new(rect.max.x, horizon + 1.0)), &[(0.0, top), (1.0, far)]);
                painter.with_clip_rect(Rect::from_min_max(Pos2::new(rect.min.x, horizon), rect.max))
            }
            None => painter.clone(),
        };

        // The water.
        let chart = self.chart.clone().filter(|_| self.show_chart);
        match &chart {
            Some((grid, id)) => self.picture.paint(ctx, &ground, cam, grid, *id, self.shallow_alarm_m, self.water_style),
            None => {
                let colour = if natural { Color32::from_rgb(16, 74, 110) } else { Color32::from_rgb(8, 22, 44) };
                ground.rect_filled(rect, 0.0, colour);
            }
        }
        if self.water_style == WaterStyle::Satellite {
            let url = self.satellite_url.clone();
            let sat = self.satellite_mut(ctx);
            sat.set_url(&url);
            sat.paint(ctx, &ground, cam, now);
        }
        self.paint_static(ctx, &ground, cam);
        if natural {
            let waves = self.waves.get_or_insert_with(|| WaveLayer::new(ctx, now));
            let land = chart.as_ref().filter(|(g, _)| layers::land_in_view(g, cam)).map(|(g, _)| g.as_ref());
            waves.paint(&ground, cam, self.sea, land, now);
            ctx.request_repaint();
        }

        {
            let mut v = vehicle.lock();
            if self.show_survey {
                self.survey.refresh(ctx, &mut v.soundings);
                self.survey.paint(&ground, cam, &v.soundings);
            }
        }
        if let Some((grid, _)) = &chart {
            if self.show_isobaths && self.view_mode == ViewMode::North {
                layers::paint_spot_depths(&ground, cam, grid);
            }
        }
        if self.show_oil {
            self.slick.update(ctx, self.oil.as_ref(), now);
            self.slick.paint(&ground, cam);
        }
        self.paint_geofence(&ground, cam);
        {
            let v = vehicle.lock();
            if self.show_track {
                self.paint_track(&ground, cam, &v.track);
            }
            self.paint_area(&ground, cam, &v.mission);
            self.paint_route(&ground, cam, &v.mission);
        }
        self.paint_home(&ground, cam);
        self.paint_target(&ground, cam);
        if self.show_vectors {
            self.paint_vectors(&ground, cam);
        }
        self.paint_boat(&ground, cam);

        match chase {
            Some(c) => self.paint_chase_extras(painter, cam, c.horizon_y as f32, now),
            None => {
                self.paint_scale(painter, cam);
                self.paint_legends(painter, rect);
            }
        }
        if self.water_style == WaterStyle::Satellite && !self.satellite_attribution.is_empty() {
            let at = Pos2::new(rect.right() - 6.0, rect.bottom() - (if self.show_fps { 30.0 } else { 4.0 }));
            let galley = painter.layout_no_wrap(self.satellite_attribution.clone(), theme::font(10.0), theme::TEXT);
            let r = Rect::from_min_size(at - galley.size() - Vec2::new(4.0, 2.0), galley.size() + Vec2::new(8.0, 4.0));
            painter.rect_filled(r, 3.0, Color32::from_rgba_unmultiplied(6, 12, 18, 170));
            painter.galley(r.min + Vec2::new(4.0, 2.0), galley, theme::TEXT);
        }
        if self.show_fps {
            let fps = self.fps();
            let text = format!("{fps:.0} fps · {:.1} ms", self.paint_ms);
            let r = Rect::from_min_size(Pos2::new(rect.right() - 150.0, rect.bottom() - 26.0), Vec2::new(142.0, 18.0));
            painter.rect_filled(r, 4.0, Color32::from_rgba_unmultiplied(6, 12, 18, 190));
            let colour = if fps >= 55.0 { theme::GOOD } else if fps >= 28.0 { theme::CAUTION } else { theme::BAD };
            painter.text(r.center(), Align2::CENTER_CENTER, text, theme::mono_bold(11.0), colour);
        }
    }

    /// The street map and the isobaths, from the cached image if it will do.
    fn paint_static(&mut self, ctx: &Context, p: &Painter, cam: &Camera) {
        if let Some(done) = self.renderer.take() {
            let texture = ctx.load_texture("static", done.image, TextureOptions::LINEAR);
            self.static_tex = Some(StaticTexture { request: done.request, texture });
            self.pending = None;
        }
        let chart = self.chart.clone().filter(|_| self.show_chart);
        let key = StaticKey {
            chart_id: chart.as_ref().map(|c| c.1).unwrap_or(0),
            shallow_alarm_m: self.shallow_alarm_m,
            isobaths: self.show_isobaths && chart.is_some(),
            osm: self.show_osm,
            natural: self.natural_water(),
            ppp: ctx.pixels_per_point(),
        };
        let v = &cam.view;
        let lon_px = v.scale * v.lon_scale();
        let view_bounds = cam.bounds();
        let covers = |r: &StaticRequest| {
            let (s, w, n, e) = r.bounds();
            s <= view_bounds.0 && w <= view_bounds.1 && view_bounds.2 <= n && view_bounds.3 <= e
        };
        let sharp = |r: &StaticRequest| (v.scale / r.scale - 1.0).abs() < 1e-9 && (lon_px / r.lon_px - 1.0).abs() < 1e-3;
        let good_enough = match &self.static_tex {
            Some(s) if s.request.key == key => (sharp(&s.request) && covers(&s.request)) || (!sharp(&s.request) && self.zooming()),
            _ => false,
        };
        let wanted = !good_enough
            && !self.pending.as_ref().is_some_and(|r| r.key == key && sharp(r) && covers(r))
            && (key.osm || key.isobaths);
        if wanted && !(self.zooming() && self.static_tex.as_ref().is_some_and(|s| s.request.key == key)) {
            // Half the size of the image: the view, any way it turns, plus spare.
            let margin = STATIC_MARGIN * v.w.max(v.h);
            let (ew, eh) = if !cam.is_chase() && v.rotation == 0.0 {
                (v.w / 2.0 + margin, v.h / 2.0 + margin)
            } else {
                let half = v.w.hypot(v.h) / 2.0 + margin;
                (half, half)
            };
            let request = StaticRequest {
                key: key.clone(),
                chart: chart.map(|c| c.0),
                north: v.centre.lat + eh / v.scale,
                west: v.centre.lon - ew / lon_px,
                scale: v.scale,
                lon_px,
                size: [2.0 * ew, 2.0 * eh],
            };
            self.pending = Some(request.clone());
            self.renderer.request(request);
        }
        if let Some(s) = &self.static_tex {
            if s.request.key.osm == key.osm || s.request.key.isobaths == key.isobaths {
                let (south, west, north, east) = s.request.bounds();
                draw::geo_image(p, cam, s.texture.id(), [north, west, south, east], Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
            }
        }
    }

    fn paint_geofence(&self, p: &Painter, cam: &Camera) {
        let (Some(home), Some(radius)) = (self.home, self.geofence_m) else { return };
        let px = radius / cam.metres_per_pixel();
        if !(10.0 < px && px < 40_000.0) {
            return;
        }
        let points = layers::geofence_points(home, radius, cam);
        draw::closed_dashed(p, cam, &points, Stroke::new(1.5, theme::alpha(theme::CAUTION, 160)), 8.0, 5.0);
    }

    fn paint_track(&self, p: &Painter, cam: &Camera, track: &[crate::vehicle::vehicle::TrackPoint]) {
        if track.len() < 2 {
            return;
        }
        let step = (track.len() / TRACK_DRAW_LIMIT).max(1);
        let mut points: Vec<[f64; 2]> = track.iter().step_by(step).map(|t| cam.view(LatLon::new(t.lat, t.lon))).collect();
        let last = track[track.len() - 1];
        points.push(cam.view(LatLon::new(last.lat, last.lon)));
        draw::polyline(p, cam, &points, Stroke::new(2.0, theme::alpha(theme::TRACK, 210)));
    }

    fn paint_area(&self, p: &Painter, cam: &Camera, mission: &Mission) {
        let Some(area) = mission.area else { return };
        let corners: Vec<[f64; 2]> = area.corners().iter().map(|c| cam.view(*c)).collect();
        draw::polygon(p, cam, &corners, theme::alpha(theme::AREA, 28), Stroke::NONE);
        draw::closed_dashed(p, cam, &corners, Stroke::new(2.0, theme::alpha(theme::AREA, 220)), 8.0, 5.0);
        if self.tool != Tool::Rect || cam.is_chase() {
            return;
        }
        let dark = Stroke::new(1.5, Color32::from_rgb(8, 19, 11));
        for c in &corners {
            draw::polygon(p, cam, &draw::rect_points(*c, 8.0), theme::AREA, dark);
        }
        let centre = cam.view(area.centre);
        draw::circle(p, cam, centre, 9.0, theme::AREA, dark);
        if let Some(handle) = Self::rotate_handle(cam, &area) {
            let top_mid = [(corners[2][0] + corners[3][0]) / 2.0, (corners[2][1] + corners[3][1]) / 2.0];
            draw::line(p, cam, top_mid, [handle.x as f64, handle.y as f64], Stroke::new(1.5, theme::AREA));
            p.circle(handle, 10.0, theme::VALUE, dark);
        }
        let label = format!("{:.0} × {:.0} m · {:.0}°", area.width_m, area.height_m, area.angle_deg);
        p.text(Pos2::new(centre[0] as f32 + 14.0, centre[1] as f32 - 12.0), Align2::LEFT_BOTTOM, label, theme::font(12.0), theme::TEXT);
    }

    fn paint_route(&self, p: &Painter, cam: &Camera, mission: &Mission) {
        let waypoints = &mission.waypoints;
        if waypoints.is_empty() {
            return;
        }
        let ground = cam.is_chase();
        let points: Vec<[f64; 2]> = waypoints.iter().map(|w| cam.view(w.position())).collect();
        let active = self.active_index.map(|a| a as i64).unwrap_or(-1);
        for i in 0..points.len().saturating_sub(1) {
            let done = active >= 0 && (i as i64 + 1) < active;
            let is_active = i as i64 + 1 == active;
            let colour = if is_active { theme::ROUTE_ACTIVE } else if done { theme::WAYPOINT_DONE } else { theme::ROUTE };
            draw::line(p, cam, points[i], points[i + 1], Stroke::new(if is_active { 3.0 } else { 1.6 }, theme::alpha(colour, 230)));
            draw::arrow(p, cam, points[i], points[i + 1], colour, 0.5, 7.0, 60.0);
            if self.show_leg_lengths && !ground {
                let (a, b) = (points[i], points[i + 1]);
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                if dx.hypot(dy) > 70.0 {
                    let length = geo::distance_m(waypoints[i].position(), waypoints[i + 1].position());
                    let mut angle = dy.atan2(dx) as f32;
                    if angle.abs() > std::f32::consts::FRAC_PI_2 {
                        angle += std::f32::consts::PI;
                    }
                    let mid = Pos2::new((a[0] + dx * 0.32) as f32, (a[1] + dy * 0.32) as f32);
                    // Above the line, as it runs.
                    let normal = Vec2::new(angle.sin(), -angle.cos()) * 9.0;
                    leg_label(p, mid + normal, angle, &format!("{length:.0}m"));
                }
            }
        }
        if mission.looped && points.len() > 2 {
            draw::dashed(p, cam, points[points.len() - 1], points[0], Stroke::new(1.2, theme::alpha(theme::ROUTE, 120)), 6.0, 4.0);
        }
        if let (Some(home), true) = (self.home, active < 0) {
            draw::dashed(p, cam, cam.view(home), points[0], Stroke::new(1.0, theme::alpha(theme::ROUTE, 90)), 2.0, 3.0);
        }
        for (i, pt) in points.iter().enumerate() {
            let selected = self.selected == Some(i);
            let is_active = i as i64 == active;
            let done = active >= 0 && (i as i64) < active;
            let radius = if selected || is_active { 12.0 } else { 10.0 };
            let fill = if is_active { theme::ROUTE_ACTIVE } else if done { theme::WAYPOINT_DONE } else { theme::WAYPOINT };
            let stroke = if selected { Stroke::new(2.5, Color32::WHITE) } else { Stroke::new(1.2, theme::INK) };
            draw::circle(p, cam, *pt, radius, fill, stroke);
            if !ground {
                p.text(Pos2::new(pt[0] as f32, pt[1] as f32), Align2::CENTER_CENTER, (i + 1).to_string(), theme::bold(11.0), theme::INK);
            }
        }
    }

    fn paint_home(&self, p: &Painter, cam: &Camera) {
        let ground = cam.is_chase();
        for (place, label, colour) in [(self.station, "GCS", theme::ACCENT), (self.home, "H", theme::HOME)] {
            let Some(place) = place else { continue };
            let at = cam.view(place);
            let size = if label == "H" { 11.0 } else { 8.0 };
            draw::polygon(p, cam, &draw::rect_points(at, size), Color32::from_rgba_unmultiplied(6, 16, 24, 210), Stroke::new(2.0, colour));
            if !ground {
                let font = if label == "H" { theme::bold(11.0) } else { theme::bold(8.0) };
                p.text(Pos2::new(at[0] as f32, at[1] as f32), Align2::CENTER_CENTER, label, font, colour);
            }
        }
    }

    fn paint_target(&self, p: &Painter, cam: &Camera) {
        let Some(target) = self.target else { return };
        let at = cam.view(target);
        for r in [7.0, 14.0] {
            draw::circle(p, cam, at, r, Color32::TRANSPARENT, Stroke::new(2.0, theme::TARGET));
        }
        if let Some((lat, lon, _)) = self.boat {
            draw::dashed(p, cam, cam.view(LatLon::new(lat, lon)), at, Stroke::new(1.2, theme::alpha(theme::TARGET, 150)), 6.0, 4.0);
        }
        if !self.target_label.is_empty() && !cam.is_chase() {
            p.text(Pos2::new(at[0] as f32 + 16.0, at[1] as f32 + 4.0), Align2::LEFT_CENTER, &self.target_label, theme::bold(11.0), theme::TARGET);
        }
    }

    /// A vector from `start` along `direction`, `length` view pixels, with an arrow head.
    #[allow(clippy::too_many_arguments)]
    fn vector(&self, p: &Painter, cam: &Camera, start: [f64; 2], direction: [f64; 2], length: f64, colour: Color32, label: &str, width: f32, dashed: bool) {
        let end = [start[0] + direction[0] * length, start[1] + direction[1] * length];
        if dashed {
            draw::dashed(p, cam, start, end, Stroke::new(width, colour), 7.0, 4.0);
        } else {
            draw::line(p, cam, start, end, Stroke::new(width, colour));
        }
        if length > 6.0 {
            draw::arrow(p, cam, start, end, colour, 1.0, 9.0, 0.0);
        }
        if !label.is_empty() && !cam.is_chase() {
            p.text(Pos2::new(end[0] as f32 + 6.0, end[1] as f32 + 4.0), Align2::LEFT_CENTER, label, theme::bold(10.5), colour);
        }
    }

    /// Thrust, true wind and current at the boat.
    fn paint_vectors(&self, p: &Painter, cam: &Camera) {
        let Some((lat, lon, heading)) = self.boat else { return };
        let boat = LatLon::new(lat, lon);
        let here = cam.view(boat);
        let v = self.vectors;
        if v.thrust.abs() > 0.02 {
            let dir = cam.view.direction(boat, heading);
            self.vector(p, cam, here, dir, 90.0 * v.thrust, theme::THRUST, &format!("T {:.0}%", v.thrust.abs() * 100.0), 5.0, false);
        }
        if v.turn.abs() > 0.03 {
            // The turning moment as an arc round the boat, starting at the bow
            // and sweeping the way the boat is being turned.
            let bow = cam.view.direction(boat, heading);
            let start = (-bow[1]).atan2(bow[0]).to_degrees();
            let sweep = -v.turn * 120.0;
            let radius = 34.0;
            let points: Vec<[f64; 2]> = (0..=24)
                .map(|k| {
                    let a = (start + sweep * k as f64 / 24.0).to_radians();
                    [here[0] + radius * a.cos(), here[1] - radius * a.sin()]
                })
                .collect();
            draw::polyline(p, cam, &points, Stroke::new(3.0, theme::alpha(theme::THRUST, 200)));
        }
        if let Some((speed, source)) = v.wind.filter(|w| w.0 > 0.2) {
            let to_source = cam.view.direction(boat, source);
            let length = (14.0 * speed).max(24.0);
            let start = [here[0] + to_source[0] * (length + 40.0), here[1] + to_source[1] * (length + 40.0)];
            self.vector(p, cam, start, [-to_source[0], -to_source[1]], length, theme::WIND, &format!("TW {speed:.1} m/s"), 3.0, false);
        }
        if let Some((drift, set)) = v.current.filter(|c| c.0 > 0.02) {
            let dir = cam.view.direction(boat, set);
            self.vector(p, cam, here, dir, (180.0 * drift).max(20.0), theme::CURRENT, &format!("Set {set:03.0}° {drift:.2}"), 3.0, true);
        }
    }

    fn paint_boat(&self, p: &Painter, cam: &Camera) {
        let Some((lat, lon, heading)) = self.boat else { return };
        let boat = LatLon::new(lat, lon);
        let here = cam.view(boat);
        let px_per_m = 1.0 / cam.metres_per_pixel();
        // Course-over-ground predictor: where the boat will be in 30 s.
        if let Some((cog, sog)) = self.boat_cog.filter(|c| c.1 > 0.15) {
            let ahead = cam.view(geo::destination(boat, cog, sog * 30.0));
            draw::line(p, cam, here, ahead, Stroke::new(2.5, theme::alpha(theme::GOOD, 220)));
            draw::circle(p, cam, ahead, 3.5, theme::GOOD, Stroke::NONE);
        }
        let dir = cam.view.direction(boat, heading);
        let reach = (25.0 * px_per_m).max(60.0);
        draw::dashed(p, cam, here, [here[0] + dir[0] * reach, here[1] + dir[1] * reach], Stroke::new(1.4, theme::alpha(theme::BOAT, 170)), 6.0, 4.0);
        if cam.is_chase() {
            return;
        }
        // The hull, to scale (never smaller than a thumbnail).
        let screen_heading = dir[0].atan2(-dir[1]);
        let length = (self.boat_length_m * px_per_m).max(30.0);
        let beam = length * (self.boat_beam_m / self.boat_length_m);
        let (s, c) = screen_heading.sin_cos();
        let pt = |x: f64, y: f64| [here[0] + x * c - y * s, here[1] + x * s + y * c];
        if self.skimmer_deployed {
            // Sweep arms: a V from the bows out to the full swath.
            let swath = (self.skimmer_swath_m * px_per_m).max(beam * 1.4);
            for side in [-1.0, 1.0] {
                draw::line(p, cam, pt(side * beam / 2.0, -length / 2.0), pt(side * swath / 2.0, -length * 1.15), Stroke::new(3.0, theme::hex(0xff5a1f)));
            }
        }
        let hull_w = beam * 0.28;
        let half = length / 2.0;
        let edge = Stroke::new(1.5, Color32::from_rgb(0x1b, 0x13, 0x00));
        for side in [-1.0, 1.0] {
            let cx = side * beam / 2.0;
            let hull = [
                pt(cx, -half),
                pt(cx + hull_w / 2.0, -half * 0.55),
                pt(cx + hull_w / 2.0, half),
                pt(cx - hull_w / 2.0, half),
                pt(cx - hull_w / 2.0, -half * 0.55),
            ];
            draw::polygon(p, cam, &hull, theme::BOAT, edge);
        }
        let deck = [pt(-beam / 2.0, -half * 0.25), pt(beam / 2.0, -half * 0.25), pt(beam / 2.0, half * 0.5), pt(-beam / 2.0, half * 0.5)];
        draw::polygon(p, cam, &deck, theme::alpha(theme::BOAT, 200), edge);
    }

    fn paint_scale(&self, p: &Painter, cam: &Camera) {
        let rect = cam.rect;
        let mpp = cam.metres_per_pixel();
        let rough = mpp * 120.0;
        let magnitude = 10f64.powf(rough.max(1e-6).log10().floor());
        let nice = [1.0, 2.0, 5.0, 10.0].iter().map(|s| magnitude * s).find(|v| *v >= rough).unwrap_or(magnitude * 10.0);
        let length = (nice / mpp) as f32;
        let (x, y) = (rect.left() + 14.0, rect.bottom() - 16.0);
        let stroke = Stroke::new(2.0, theme::TEXT);
        p.line_segment([Pos2::new(x, y), Pos2::new(x + length, y)], stroke);
        p.line_segment([Pos2::new(x, y - 5.0), Pos2::new(x, y + 3.0)], stroke);
        p.line_segment([Pos2::new(x + length, y - 5.0), Pos2::new(x + length, y + 3.0)], stroke);
        p.text(Pos2::new(x + length + 8.0, y), Align2::LEFT_CENTER, layers::nice_length(nice), theme::bold(12.0), theme::TEXT);
        if self.view_mode == ViewMode::Heading {
            // A north arrow, since north is no longer up.
            let centre = Pos2::new(rect.right() - 30.0 - SIDE_TAB_W, rect.top() + 104.0);
            let d = cam.view.direction(cam.view.centre, 0.0);
            let tip = centre + Vec2::new(d[0] as f32, d[1] as f32) * 16.0;
            p.line_segment([centre, tip], Stroke::new(3.0, theme::BAD));
            p.text(tip + Vec2::new(d[0] as f32, d[1] as f32) * 9.0, Align2::CENTER_CENTER, "N", theme::bold(12.0), theme::TEXT);
        }
    }

    fn paint_legends(&self, p: &Painter, rect: Rect) {
        if self.show_survey && self.survey.has_data() {
            let (low, high) = self.survey.depth_range();
            layers::paint_survey_legend(p, Rect::from_min_size(Pos2::new(rect.right() - 34.0 - SIDE_TAB_W, rect.top() + 150.0), Vec2::new(12.0, 120.0)), low, high, "survey m");
        }
        if !self.show_vectors || self.boat.is_none() {
            return;
        }
        let items = [
            (theme::BOAT, "Heading"),
            (theme::GOOD, "COG/SOG 30 s"),
            (theme::THRUST, "Thrust"),
            (theme::WIND, "True wind"),
            (theme::CURRENT, "Current"),
        ];
        // Clear of the side tab on the chart's right edge.
        let r = Rect::from_min_size(Pos2::new(rect.right() - 128.0 - SIDE_TAB_W, rect.top() + 286.0), Vec2::new(118.0, 16.0 * items.len() as f32 + 8.0));
        p.rect_filled(r, 6.0, Color32::from_rgba_unmultiplied(6, 12, 18, 190));
        for (i, (colour, text)) in items.iter().enumerate() {
            let y = r.top() + 12.0 + i as f32 * 16.0;
            p.line_segment([Pos2::new(r.left() + 8.0, y), Pos2::new(r.left() + 26.0, y)], Stroke::new(3.0, *colour));
            p.text(Pos2::new(r.left() + 32.0, y), Align2::LEFT_CENTER, *text, theme::font(10.0), theme::TEXT);
        }
    }

    // ------------------------------------------------------------------
    // The chase camera's sky, haze, sun, tags and boat
    // ------------------------------------------------------------------

    /// Roughly where the sun is from the local clock: east in the morning,
    /// south at about one o'clock, west in the evening.
    pub fn sun_azimuth(hour: f64) -> f64 {
        let hour = hour.clamp(6.0, 20.0);
        (180.0 + 15.0 * (hour - 13.0)).rem_euclid(360.0)
    }

    fn paint_chase_extras(&self, p: &Painter, cam: &Camera, horizon: f32, now: f64) {
        let rect = cam.rect;
        let (w, h) = (rect.width(), rect.height());
        let natural = self.natural_water();
        let far = if natural { SKY_HORIZON } else { Color32::from_rgb(70, 110, 150) };
        // Haze towards the horizon, which is what sells the distance.  Over
        // natural water it reaches further: far off, water mirrors the sky.
        let reach = if natural { 0.38 } else { 0.22 };
        let haze = Rect::from_min_size(Pos2::new(rect.left(), horizon), Vec2::new(w, h * reach));
        draw::vertical_gradient(p, haze, &[(0.0, theme::alpha(far, 235)), (0.35, theme::alpha(far, 110)), (1.0, theme::alpha(far, 0))]);
        if natural {
            self.paint_sun(p, rect, horizon, now);
        }

        // Upright tags over the places that matter, like markers in a game.
        let mut tags = Vec::new();
        if let Some(t) = self.target {
            tags.push((t, if self.target_label.is_empty() { "TARGET".to_string() } else { self.target_label.clone() }, theme::TARGET));
        }
        if let Some(home) = self.home {
            tags.push((home, "HOME".to_string(), theme::HOME));
        }
        for (place, text, colour) in tags {
            let Some(at) = cam.project(place) else { continue };
            if !(horizon < at.y && at.y < rect.bottom() && rect.left() - 40.0 < at.x && at.x < rect.right() + 40.0) {
                continue;
            }
            let galley = p.layout_no_wrap(text, theme::bold(12.0), colour);
            let size = galley.size() + Vec2::new(14.0, 6.0);
            let r = Rect::from_center_size(Pos2::new(at.x, at.y - 30.0), size);
            p.rect(r, 5.0, Color32::from_rgba_unmultiplied(6, 12, 18, 210), Stroke::new(1.5, colour), egui::StrokeKind::Middle);
            p.line_segment([Pos2::new(at.x, r.bottom()), at], Stroke::new(1.5, colour));
            p.galley(r.center() - galley.size() / 2.0, galley, colour);
        }
        if let Some((lat, lon, _)) = self.boat {
            if let Some(at) = cam.project(LatLon::new(lat, lon)) {
                self.paint_boat_3d(p, at);
            }
        }
        p.text(
            Pos2::new(rect.right() - 10.0, horizon - 22.0),
            Align2::RIGHT_TOP,
            format!("CHASE CAM · {:.0} m", self.view_width_m()),
            theme::bold(12.0),
            theme::TEXT,
        );
    }

    /// The sun over the horizon when the camera faces it, and its glitter on
    /// the water: a path of sparkles widening towards the camera.
    fn paint_sun(&self, p: &Painter, rect: Rect, horizon: f32, now: f64) {
        use chrono::Timelike;
        let t = chrono::Local::now();
        let hour = t.hour() as f64 + t.minute() as f64 / 60.0;
        let off = (Self::sun_azimuth(hour) - self.rotation + 540.0).rem_euclid(360.0) - 180.0;
        if off.abs() > 60.0 {
            return;
        }
        let (w, h) = (rect.width(), rect.height());
        let sx = rect.left() + w / 2.0 + (off / 45.0) as f32 * w * 0.5;
        let sy = horizon - h * 0.075;
        let glow = h * 0.16;
        let sky = p.with_clip_rect(Rect::from_min_max(rect.min, Pos2::new(rect.right(), horizon + 1.0)));
        draw::radial_gradient(
            &sky,
            Pos2::new(sx, sy),
            Vec2::splat(glow),
            &[
                (0.0, Color32::from_rgba_unmultiplied(255, 250, 225, 255)),
                (0.08, Color32::from_rgba_unmultiplied(255, 244, 210, 240)),
                (0.25, Color32::from_rgba_unmultiplied(255, 230, 180, 90)),
                (1.0, Color32::from_rgba_unmultiplied(255, 220, 170, 0)),
            ],
        );
        let water = p.with_clip_rect(Rect::from_min_max(Pos2::new(rect.left(), horizon), rect.max));
        // The bright band under the sun.
        draw::radial_gradient(
            &water,
            Pos2::new(sx, horizon + 4.0),
            Vec2::new(w * 0.12, h * 0.05),
            &[(0.0, Color32::from_rgba_unmultiplied(255, 245, 220, 150)), (1.0, Color32::from_rgba_unmultiplied(255, 245, 220, 0))],
        );
        // Sparkles, reshuffled eight times a second: flat dashes, bigger
        // nearer the camera.
        let mut rng = Rng::new((now * 8.0) as u64);
        for _ in 0..160 {
            let u = rng.random().powf(1.7) as f32;
            let y = horizon + 2.0 + u * h * 0.62;
            let centre = sx + (rect.left() + w / 2.0 - sx) * u * 0.6;
            let x = centre + rng.gauss(0.0, 1.0) as f32 * (8.0 + u * w * 0.09);
            let (wd, ht) = (1.5 + u * 5.5, 0.8 + u * 1.6);
            let bright = rng.uniform(0.4, 1.0) as f32 * (1.0 - u * 0.5);
            let alpha = if bright < 0.45 { 110 } else if bright < 0.7 { 170 } else { 235 };
            water.rect_filled(Rect::from_center_size(Pos2::new(x, y), Vec2::new(wd, ht)), 0.0, Color32::from_rgba_unmultiplied(255, 250, 235, alpha));
        }
    }

    /// The catamaran from astern, scaled with distance, heeling with the roll.
    fn paint_boat_3d(&self, p: &Painter, at: Pos2) {
        // True size at the boat's distance (about twice the chart scale
        // there), but never so small it cannot be seen or so big it hides the water.
        let true_px = self.boat_length_m / self.metres_per_pixel() * 2.0;
        let size = (true_px * 0.9).clamp(80.0, 170.0) as f32;
        let s = size / 100.0;
        let (sin, cos) = (self.boat_roll as f32).to_radians().sin_cos();
        let tr = |q: Pos2| Pos2::new(at.x + (q.x * cos - q.y * sin) * s, at.y + (q.x * sin + q.y * cos) * s);
        let poly = |points: Vec<Pos2>, fill: Color32| {
            p.add(Shape::convex_polygon(points.into_iter().map(tr).collect(), fill, Stroke::NONE));
        };
        let ellipse = |c: Pos2, r: Vec2| (0..24).map(move |i| {
            let a = std::f32::consts::TAU * i as f32 / 24.0;
            Pos2::new(c.x + r.x * a.cos(), c.y + r.y * a.sin())
        });
        poly(ellipse(Pos2::new(0.0, 8.0), Vec2::new(62.0, 12.0)).collect(), Color32::from_rgba_unmultiplied(0, 0, 0, 90));
        // Wake.
        for side in [-1.0f32, 1.0] {
            poly(
                vec![Pos2::new(side * 40.0, 4.0), Pos2::new(side * 75.0, 34.0), Pos2::new(side * 22.0, 34.0)],
                Color32::from_rgba_unmultiplied(230, 245, 255, 70),
            );
        }
        let hull = theme::BOAT;
        let dark = theme::darker(hull, 1.7);
        for side in [-1.0f32, 1.0] {
            let cx = side * 38.0;
            poly(draw::rounded_rect_points(Rect::from_min_size(Pos2::new(cx - 13.0, -10.0), Vec2::new(26.0, 18.0)), 6.0), dark);
            poly(draw::rounded_rect_points(Rect::from_min_size(Pos2::new(cx - 13.0, -22.0), Vec2::new(26.0, 16.0)), 6.0), hull);
        }
        let grey = theme::hex(0xd6dde4);
        poly(draw::rounded_rect_points(Rect::from_min_size(Pos2::new(-44.0, -30.0), Vec2::new(88.0, 10.0)), 0.5), grey);
        poly(draw::rounded_rect_points(Rect::from_min_size(Pos2::new(-22.0, -52.0), Vec2::new(44.0, 23.0)), 4.0), theme::hex(0x2b3846));
        p.line_segment([tr(Pos2::new(0.0, -52.0)), tr(Pos2::new(0.0, -84.0))], Stroke::new((2.5 * s).max(1.0), grey));
        p.circle_filled(tr(Pos2::new(0.0, -86.0)), 4.0 * s, theme::GOOD);
    }
}

/// A leg length, turned to run along its leg.
fn leg_label(p: &Painter, centre: Pos2, angle: f32, text: &str) {
    let galley = p.layout_no_wrap(text.to_string(), theme::font(11.0), theme::TEXT);
    let pos = centre - galley.size() / 2.0;
    p.add(TextShape::new(pos, galley, theme::TEXT).with_angle_and_anchor(angle, Align2::CENTER_CENTER));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sun_follows_the_clock() {
        assert!((ChartView::sun_azimuth(13.0) - 180.0).abs() < 1e-9);
        assert!(ChartView::sun_azimuth(8.0) < 180.0);
        assert!(ChartView::sun_azimuth(18.0) > 180.0);
    }
}
