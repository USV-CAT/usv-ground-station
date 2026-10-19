//! Routes: waypoints, and the rectangle patterns that generate them.
//!
//! A mission is an ordered list of waypoints.  It can be clicked onto the
//! chart point by point, or generated from a rectangle: a *lawnmower* sweeps
//! it in parallel lanes, which is how a single-beam echo sounder surveys an
//! area, and a *perimeter* runs round its edge.
//!
//! The rectangle is kept with the mission so a survey can be regenerated
//! with a different lane spacing without drawing it again.  The file format
//! is the one the Python station wrote, so old missions still open.

use std::path::Path;

use serde_json::{json, Value};

use super::geo::{self, LatLon, Metres};

#[derive(Clone, Debug, PartialEq)]
pub struct Waypoint {
    pub lat: f64,
    pub lon: f64,
    /// Speed for the leg that ends here, m/s.  None uses the cruise speed.
    pub speed_ms: Option<f64>,
    /// Station-keep here this long before moving on, seconds.
    pub hold_s: f64,
}

impl Waypoint {
    pub fn new(lat: f64, lon: f64) -> Self {
        Waypoint { lat, lon, speed_ms: None, hold_s: 0.0 }
    }

    pub fn position(&self) -> LatLon {
        LatLon::new(self.lat, self.lon)
    }

    fn to_json(&self) -> Value {
        let round = |v: f64| (v * 1e8).round() / 1e8;
        let mut data = json!({"lat": round(self.lat), "lon": round(self.lon)});
        if let Some(speed) = self.speed_ms {
            data["speed_ms"] = json!(speed);
        }
        if self.hold_s != 0.0 {
            data["hold_s"] = json!(self.hold_s);
        }
        data
    }

    fn from_json(data: &Value) -> Option<Waypoint> {
        Some(Waypoint {
            lat: number(data.get("lat")?)?,
            lon: number(data.get("lon")?)?,
            speed_ms: data.get("speed_ms").and_then(number),
            hold_s: data.get("hold_s").and_then(number).unwrap_or(0.0),
        })
    }
}

fn number(v: &Value) -> Option<f64> {
    v.as_f64().or_else(|| v.as_str().and_then(|s| s.trim().parse().ok()))
}

/// A rectangle on the water, possibly rotated.
///
/// `angle_deg` is the bearing of the rectangle's *width* axis, so 90° is a
/// rectangle squared to the compass with its width running east.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rectangle {
    pub centre: LatLon,
    pub width_m: f64,
    pub height_m: f64,
    pub angle_deg: f64,
}

/// Unit vectors (east, north) along the width and the height of a rectangle.
fn axes(angle_deg: f64) -> ((f64, f64), (f64, f64)) {
    let theta = angle_deg.to_radians();
    let u = (theta.sin(), theta.cos());
    let v = (-u.1, u.0);
    (u, v)
}

impl Rectangle {
    pub fn new(centre: LatLon, width_m: f64, height_m: f64, angle_deg: f64) -> Self {
        Rectangle { centre, width_m, height_m, angle_deg }
    }

    /// The rectangle with opposite corners `a` and `b`, at `angle_deg`.
    pub fn from_corners(a: LatLon, b: LatLon, angle_deg: f64) -> Rectangle {
        let mid = LatLon::new((a.lat + b.lat) / 2.0, (a.lon + b.lon) / 2.0);
        let offset = geo::to_metres(a, b);
        let (u, v) = axes(angle_deg);
        let width = (offset.east * u.0 + offset.north * u.1).abs();
        let height = (offset.east * v.0 + offset.north * v.1).abs();
        Rectangle::new(mid, width, height, angle_deg.rem_euclid(360.0))
    }

    /// The point `along` the width axis and `across` it from the centre.
    pub fn point(&self, along: f64, across: f64) -> LatLon {
        let (u, v) = axes(self.angle_deg);
        geo::to_lat_lon(self.centre, Metres::new(along * u.0 + across * v.0, along * u.1 + across * v.1))
    }

    pub fn corners(&self) -> [LatLon; 4] {
        let (w, h) = (self.width_m / 2.0, self.height_m / 2.0);
        [self.point(-w, -h), self.point(w, -h), self.point(w, h), self.point(-w, h)]
    }

    pub fn contains(&self, point: LatLon) -> bool {
        let local = geo::to_metres(self.centre, point);
        let (u, v) = axes(self.angle_deg);
        let along = local.east * u.0 + local.north * u.1;
        let across = local.east * v.0 + local.north * v.1;
        along.abs() <= self.width_m / 2.0 && across.abs() <= self.height_m / 2.0
    }

    pub fn area_m2(&self) -> f64 {
        self.width_m * self.height_m
    }

    fn to_json(self) -> Value {
        json!({
            "lat": self.centre.lat, "lon": self.centre.lon,
            "width_m": self.width_m, "height_m": self.height_m, "angle_deg": self.angle_deg,
        })
    }

    fn from_json(data: &Value) -> Option<Rectangle> {
        Some(Rectangle::new(
            LatLon::new(number(data.get("lat")?)?, number(data.get("lon")?)?),
            number(data.get("width_m")?)?,
            number(data.get("height_m")?)?,
            data.get("angle_deg").and_then(number).unwrap_or(90.0),
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pattern {
    /// Parallel lanes, back and forth (a boustrophedon).
    Lawnmower,
    /// Lanes one way, then across them: a crosshatch, for a denser survey.
    Grid,
    /// A rectangular spiral from the edge in to the middle.
    SpiralIn,
    /// A rectangular spiral from the middle out to the edge.
    SpiralOut,
    /// Round the edge.
    Perimeter,
    /// The planner tries the others from every corner and picks the quickest
    /// safe one.  In development.
    Auto,
}

impl Pattern {
    pub const ALL: [Pattern; 6] =
        [Pattern::Lawnmower, Pattern::Grid, Pattern::SpiralIn, Pattern::SpiralOut, Pattern::Perimeter, Pattern::Auto];

    pub fn as_str(self) -> &'static str {
        match self {
            Pattern::Lawnmower => "lawnmower",
            Pattern::Grid => "grid",
            Pattern::SpiralIn => "spiral_in",
            Pattern::SpiralOut => "spiral_out",
            Pattern::Perimeter => "perimeter",
            Pattern::Auto => "auto",
        }
    }

    pub fn parse(text: &str) -> Pattern {
        Pattern::ALL.into_iter().find(|p| p.as_str() == text).unwrap_or(Pattern::Lawnmower)
    }

    pub fn label(self) -> &'static str {
        match self {
            Pattern::Lawnmower => "Lawnmower",
            Pattern::Grid => "Grid",
            Pattern::SpiralIn => "Spiral in",
            Pattern::SpiralOut => "Spiral out",
            Pattern::Perimeter => "Perimeter",
            Pattern::Auto => "AI · beta",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Pattern::Lawnmower => "Parallel lanes back and forth — the standard echo-sounder survey.",
            Pattern::Grid => "Lanes one way, then across them: every spot is crossed twice.",
            Pattern::SpiralIn => "A spiral from the outside in to the centre: the edge first.",
            Pattern::SpiralOut => "A spiral from the centre out to the edge: the middle first.",
            Pattern::Perimeter => "Round the edge of the area, as many laps as you set.",
            Pattern::Auto => "In development: tries every pattern from every corner and picks the quickest that keeps off the shallows.",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PatternSpec {
    pub pattern: Pattern,
    /// Distance between lanes, metres.  Worked out from the swath and the
    /// overlap when those are set.
    pub spacing_m: f64,
    /// Distance kept inside the rectangle's edge, metres: the offset.
    pub margin_m: f64,
    /// Lanes run along the longer side, which means fewer turns.
    pub along_long_side: bool,
    /// Laps of a perimeter pattern.
    pub laps: u32,
    /// Width covered by one pass (sensor swath or skimmer arms), metres.
    pub swath_m: f64,
    /// How much neighbouring passes overlap, percent of the swath.
    pub overlap_pct: f64,
    /// The corner the route starts from, in the order of [`Rectangle::corners`].
    pub start_corner: u8,
}

impl Default for PatternSpec {
    fn default() -> Self {
        PatternSpec {
            pattern: Pattern::Lawnmower,
            spacing_m: 10.0,
            margin_m: 2.0,
            along_long_side: true,
            laps: 1,
            swath_m: 10.0,
            overlap_pct: 0.0,
            start_corner: 0,
        }
    }
}

/// Lane spacing for a swath with `overlap_pct` overlap between neighbours.
pub fn spacing_for(swath_m: f64, overlap_pct: f64) -> f64 {
    (swath_m * (1.0 - overlap_pct.clamp(0.0, 95.0) / 100.0)).max(0.5)
}

impl PatternSpec {
    /// Set the lane spacing from the swath and the overlap.
    pub fn apply_overlap(&mut self) {
        self.spacing_m = spacing_for(self.swath_m, self.overlap_pct);
    }

    fn to_json(self) -> Value {
        json!({
            "pattern": self.pattern.as_str(), "spacing_m": self.spacing_m, "margin_m": self.margin_m,
            "along_long_side": self.along_long_side, "laps": self.laps,
            "swath_m": self.swath_m, "overlap_pct": self.overlap_pct, "start_corner": self.start_corner,
        })
    }

    fn from_json(data: &Value) -> PatternSpec {
        let mut spec = PatternSpec::default();
        if let Some(p) = data.get("pattern").and_then(Value::as_str) {
            spec.pattern = Pattern::parse(p);
        }
        if let Some(v) = data.get("spacing_m").and_then(number) {
            spec.spacing_m = v;
        }
        if let Some(v) = data.get("margin_m").and_then(number) {
            spec.margin_m = v;
        }
        if let Some(v) = data.get("along_long_side").and_then(Value::as_bool) {
            spec.along_long_side = v;
        }
        if let Some(v) = data.get("laps").and_then(number) {
            spec.laps = v.max(1.0) as u32;
        }
        // Files from before the overlap setting: the spacing is the swath.
        spec.swath_m = data.get("swath_m").and_then(number).unwrap_or(spec.spacing_m);
        spec.overlap_pct = data.get("overlap_pct").and_then(number).unwrap_or(0.0);
        spec.start_corner = data.get("start_corner").and_then(number).map(|v| (v as u8) % 4).unwrap_or(0);
        spec
    }
}

// --------------------------------------------------------------------------
// The patterns, in the rectangle's own frame
// --------------------------------------------------------------------------
//
// Each pattern is worked out as (along, across) metres from the rectangle's
// centre — along its width axis and across it — starting at the (-, -)
// corner, then flipped to start from the chosen corner.

type Local = (f64, f64);

/// Parallel lanes across a w × h box (margin already taken off), alternating.
fn lawnmower_local(half_w: f64, half_h: f64, spacing: f64, lanes_along_width: bool) -> Vec<Local> {
    let (run, span) = if lanes_along_width { (half_w, half_h) } else { (half_h, half_w) };
    let lanes = if span > 0.0 { ((2.0 * span / spacing).ceil() as usize + 1).max(1) } else { 1 };
    let step = if lanes > 1 { 2.0 * span / (lanes - 1) as f64 } else { 0.0 };
    let mut points = Vec::with_capacity(lanes * 2);
    for index in 0..lanes {
        let offset = -span + index as f64 * step;
        let (start, end) = if index % 2 == 0 { (-run, run) } else { (run, -run) };
        for along in [start, end] {
            points.push(if lanes_along_width { (along, offset) } else { (offset, along) });
        }
    }
    points
}

/// A rectangular spiral from the (-a, -b) corner in to the middle, its laps
/// `step` apart.
fn spiral_in_local(a: f64, b: f64, step: f64) -> Vec<Local> {
    let (mut x0, mut y0, mut x1, mut y1) = (-a, -b, a, b);
    let mut points = vec![(x0, y0)];
    loop {
        if x1 <= x0 || y1 < y0 {
            break;
        }
        points.push((x1, y0));
        if y1 <= y0 {
            break;
        }
        points.push((x1, y1));
        points.push((x0, y1));
        let next_y0 = y0 + step;
        if next_y0 >= y1 - 1e-9 {
            break;
        }
        points.push((x0, next_y0));
        x0 += step;
        y0 = next_y0;
        x1 -= step;
        y1 -= step;
        if x1 < x0 {
            break;
        }
    }
    points
}

fn perimeter_local(a: f64, b: f64, laps: u32) -> Vec<Local> {
    let corners = [(-a, -b), (a, -b), (a, b), (-a, b)];
    let mut points = Vec::new();
    for _ in 0..laps.max(1) {
        points.extend_from_slice(&corners);
    }
    points.push(corners[0]);
    points
}

/// Flip a route that starts at the (-, -) corner to start at `corner`.
fn from_corner(points: Vec<Local>, corner: u8) -> Vec<Local> {
    let sx = if matches!(corner % 4, 1 | 2) { -1.0 } else { 1.0 };
    let sy = if matches!(corner % 4, 2 | 3) { -1.0 } else { 1.0 };
    points.into_iter().map(|(x, y)| (x * sx, y * sy)).collect()
}

fn to_geo(rect: &Rectangle, points: &[Local]) -> Vec<LatLon> {
    points.iter().map(|&(along, across)| rect.point(along, across)).collect()
}

fn half_extents(rect: &Rectangle, margin: f64) -> (f64, f64) {
    ((rect.width_m / 2.0 - margin).max(0.0), (rect.height_m / 2.0 - margin).max(0.0))
}

/// Parallel lanes across `rect`, alternating direction (a boustrophedon).
///
/// Lanes are spread evenly so the outer two sit exactly `margin_m` inside the
/// edge, which means the actual spacing is at most `spacing_m`.
pub fn lawnmower(rect: &Rectangle, spacing_m: f64, margin_m: f64, along_long_side: bool) -> Vec<LatLon> {
    let (a, b) = half_extents(rect, margin_m);
    let lanes_along_width = (rect.width_m >= rect.height_m) == along_long_side;
    to_geo(rect, &lawnmower_local(a, b, spacing_m.max(0.01), lanes_along_width))
}

/// Round the edge of `rect`, `laps` times, finishing where it started.
pub fn perimeter(rect: &Rectangle, margin_m: f64, laps: u32) -> Vec<LatLon> {
    let (a, b) = half_extents(rect, margin_m);
    to_geo(rect, &perimeter_local(a, b, laps))
}

/// A spiral from the edge in to the middle, laps `spacing_m` apart.
pub fn spiral_in(rect: &Rectangle, spacing_m: f64, margin_m: f64) -> Vec<LatLon> {
    let (a, b) = half_extents(rect, margin_m);
    to_geo(rect, &spiral_in_local(a, b, spacing_m.max(0.01)))
}

/// A spiral from the middle out to the edge.
pub fn spiral_out(rect: &Rectangle, spacing_m: f64, margin_m: f64) -> Vec<LatLon> {
    let mut route = spiral_in(rect, spacing_m, margin_m);
    route.reverse();
    route
}

fn generate_local(rect: &Rectangle, spec: &PatternSpec, pattern: Pattern) -> Vec<Local> {
    let (a, b) = half_extents(rect, spec.margin_m);
    let spacing = spec.spacing_m.max(0.01);
    let lanes_along_width = (rect.width_m >= rect.height_m) == spec.along_long_side;
    let route = match pattern {
        Pattern::Lawnmower | Pattern::Auto => lawnmower_local(a, b, spacing, lanes_along_width),
        Pattern::Grid => {
            let mut first = lawnmower_local(a, b, spacing, lanes_along_width);
            let end = *first.last().unwrap();
            // The cross lanes start from the corner nearest where the first set ended.
            let second = lawnmower_local(a, b, spacing, !lanes_along_width);
            let corner = (0u8..4)
                .min_by(|&x, &y| {
                    let d = |c: u8| {
                        let p = from_corner(vec![second[0]], c)[0];
                        (p.0 - end.0).hypot(p.1 - end.1)
                    };
                    d(x).total_cmp(&d(y))
                })
                .unwrap();
            first.extend(from_corner(second, corner));
            first
        }
        Pattern::SpiralIn => spiral_in_local(a, b, spacing),
        Pattern::SpiralOut => {
            let mut r = spiral_in_local(a, b, spacing);
            r.reverse();
            r
        }
        Pattern::Perimeter => perimeter_local(a, b, spec.laps),
    };
    from_corner(route, spec.start_corner)
}

/// The route for a pattern.  [`Pattern::Auto`] without context plans as if
/// from the middle of the area with no chart; see [`plan_auto`].
pub fn generate(rect: &Rectangle, spec: &PatternSpec) -> Vec<LatLon> {
    match spec.pattern {
        Pattern::Auto => plan_auto(rect, spec, &AutoContext::default()).route,
        p => to_geo(rect, &generate_local(rect, spec, p)),
    }
}

// --------------------------------------------------------------------------
// The planner ("AI", in development)
// --------------------------------------------------------------------------

/// What the planner knows about the world.
pub struct AutoContext<'a> {
    /// Where the boat is now: the transit to the first point counts.
    pub start: Option<LatLon>,
    pub cruise_ms: f64,
    /// Charted depth, metres (NaN on land).
    pub depth: Option<&'a dyn Fn(f64, f64) -> f64>,
    /// Water shallower than this is to be kept off.
    pub shallow_m: f64,
}

impl Default for AutoContext<'_> {
    fn default() -> Self {
        AutoContext { start: None, cruise_ms: 1.6, depth: None, shallow_m: 1.2 }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AutoPlan {
    pub route: Vec<LatLon>,
    /// The pattern and corner the planner chose.
    pub chosen: PatternSpec,
    /// Its estimate of the time the route takes, seconds.
    pub seconds: f64,
    /// Route samples in water shallower than the limit (or on land).
    pub shallow_samples: usize,
    pub summary: String,
}

/// Seconds a turn of `degrees` costs a small catamaran: it slows, pivots and
/// settles onto the new line.
fn turn_cost(degrees: f64) -> f64 {
    8.0 * degrees.abs() / 180.0
}

/// Score a route: time to run it, plus heavy penalties for shallow water.
fn score(route: &[LatLon], ctx: &AutoContext) -> (f64, usize, usize) {
    let cruise = ctx.cruise_ms.max(0.2);
    let mut seconds = geo::path_length_m(route) / cruise;
    if let (Some(start), Some(first)) = (ctx.start, route.first()) {
        seconds += geo::distance_m(start, *first) / cruise;
    }
    let mut turns = 0;
    for w in route.windows(3) {
        let turn = geo::angle_difference(geo::bearing_deg(w[1], w[2]), geo::bearing_deg(w[0], w[1]));
        if turn.abs() > 20.0 {
            turns += 1;
        }
        seconds += turn_cost(turn);
    }
    let mut shallow = 0;
    if let Some(depth) = ctx.depth {
        for w in route.windows(2) {
            let leg = geo::distance_m(w[0], w[1]);
            let n = ((leg / 15.0).ceil() as usize).max(1);
            let bearing = geo::bearing_deg(w[0], w[1]);
            for i in 0..n {
                let p = geo::destination(w[0], bearing, leg * i as f64 / n as f64);
                let d = depth(p.lat, p.lon);
                if d.is_nan() || d < ctx.shallow_m {
                    shallow += 1;
                }
            }
        }
        seconds += shallow as f64 * 600.0;
    }
    (seconds, shallow, turns)
}

/// Try the lane and spiral patterns from every corner, and keep the one that
/// is quickest to run — counting the transit from the boat and the turns —
/// while keeping off charted shallows.  An honest heuristic, not a learned
/// model: the "AI" mode is in development.
pub fn plan_auto(rect: &Rectangle, spec: &PatternSpec, ctx: &AutoContext) -> AutoPlan {
    let mut best: Option<(f64, usize, usize, PatternSpec, Vec<LatLon>)> = None;
    for pattern in [Pattern::Lawnmower, Pattern::SpiralIn, Pattern::SpiralOut] {
        let sides: &[bool] = if pattern == Pattern::Lawnmower { &[true, false] } else { &[true] };
        for &along_long_side in sides {
            for corner in 0..4u8 {
                let candidate = PatternSpec { pattern, along_long_side, start_corner: corner, ..*spec };
                let route = to_geo(rect, &generate_local(rect, &candidate, pattern));
                let (seconds, shallow, turns) = score(&route, ctx);
                if best.as_ref().is_none_or(|b| seconds < b.0) {
                    best = Some((seconds, shallow, turns, candidate, route));
                }
            }
        }
    }
    let (seconds, shallow, turns, chosen, route) = best.unwrap();
    let corner = rect.corners()[chosen.start_corner as usize % 4];
    let from = super::units::compass_point(geo::bearing_deg(rect.centre, corner));
    let what = match chosen.pattern {
        Pattern::Lawnmower if chosen.along_long_side => "lanes along the long side".to_string(),
        Pattern::Lawnmower => "lanes along the short side".to_string(),
        p => p.label().to_lowercase(),
    };
    // A spiral out begins in the middle and finishes at its corner.
    let corner_text = if chosen.pattern == Pattern::SpiralOut { format!("finishing at the {from} corner") } else { format!("from the {from} corner") };
    let mut summary = format!(
        "Chose {what} {corner_text}: {}, {turns} turns, about {}",
        super::units::format_distance(Some(geo::path_length_m(&route))),
        super::units::format_duration(Some(seconds - shallow as f64 * 600.0)),
    );
    if shallow > 0 {
        summary.push_str(&format!(" — warning: {shallow} points still in shallow water"));
    }
    AutoPlan { route, chosen, seconds, shallow_samples: shallow, summary }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Mission {
    pub waypoints: Vec<Waypoint>,
    pub name: String,
    /// Start again from the first waypoint after the last one.
    pub looped: bool,
    /// The rectangle this route was generated from, if it was.
    pub area: Option<Rectangle>,
    pub pattern: PatternSpec,
    /// Move the whole route with the water at this velocity, (north, east)
    /// m/s.  A sweep of an oil slick is planned relative to the oil.
    pub drift_ms: Option<(f64, f64)>,
}

impl Default for Mission {
    fn default() -> Self {
        Mission {
            waypoints: Vec::new(),
            name: "mission".into(),
            looped: false,
            area: None,
            pattern: PatternSpec::default(),
            drift_ms: None,
        }
    }
}

impl Mission {
    pub fn positions(&self) -> Vec<LatLon> {
        self.waypoints.iter().map(Waypoint::position).collect()
    }

    pub fn add(&mut self, lat: f64, lon: f64) -> &mut Waypoint {
        self.waypoints.push(Waypoint::new(lat, lon));
        self.waypoints.last_mut().unwrap()
    }

    pub fn remove(&mut self, index: usize) {
        if index < self.waypoints.len() {
            self.waypoints.remove(index);
        }
    }

    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.area = None;
        self.drift_ms = None;
    }

    pub fn set_route(&mut self, points: &[LatLon], speed_ms: Option<f64>) {
        self.waypoints =
            points.iter().map(|p| Waypoint { lat: p.lat, lon: p.lon, speed_ms, hold_s: 0.0 }).collect();
    }

    /// Rebuild the route from the stored rectangle.  Returns whether it did.
    pub fn regenerate(&mut self) -> bool {
        let Some(area) = self.area else { return false };
        let route = generate(&area, &self.pattern);
        self.set_route(&route, None);
        true
    }

    /// Translate the route and its area, as the water carries them.
    pub fn shift(&mut self, north_m: f64, east_m: f64) {
        let offset = Metres::new(east_m, north_m);
        for w in &mut self.waypoints {
            let moved = geo::to_lat_lon(w.position(), offset);
            w.lat = moved.lat;
            w.lon = moved.lon;
        }
        if let Some(area) = &mut self.area {
            area.centre = geo::to_lat_lon(area.centre, offset);
        }
    }

    pub fn length_m(&self, start: Option<LatLon>) -> f64 {
        let mut points: Vec<LatLon> = start.into_iter().collect();
        let positions = self.positions();
        points.extend_from_slice(&positions);
        if self.looped && positions.len() > 1 {
            points.push(positions[0]);
        }
        geo::path_length_m(&points)
    }

    /// Time to run the route at cruise speed, plus holds and a turn allowance.
    pub fn duration_s(&self, cruise_speed_ms: f64, start: Option<LatLon>) -> f64 {
        if cruise_speed_ms <= 0.0 {
            return f64::INFINITY;
        }
        let turns = self.waypoints.len().saturating_sub(1) as f64 * 4.0;
        let holds: f64 = self.waypoints.iter().map(|w| w.hold_s).sum();
        self.length_m(start) / cruise_speed_ms + turns + holds
    }

    // -- files ----------------------------------------------------------------

    pub fn to_json(&self) -> Value {
        json!({
            "format": "usvgs-mission",
            "version": 1,
            "name": self.name,
            "loop": self.looped,
            "area": self.area.map(Rectangle::to_json),
            "pattern": self.pattern.to_json(),
            "drift_ms": self.drift_ms.map(|(n, e)| json!([n, e])),
            "waypoints": self.waypoints.iter().map(Waypoint::to_json).collect::<Vec<_>>(),
        })
    }

    pub fn from_json(data: &Value) -> Result<Mission, String> {
        if !data.is_object() {
            return Err("not a mission file".into());
        }
        let waypoints = match data.get("waypoints") {
            Some(Value::Array(items)) => items
                .iter()
                .map(|w| Waypoint::from_json(w).ok_or_else(|| format!("bad waypoint {w}")))
                .collect::<Result<Vec<_>, _>>()?,
            _ => Vec::new(),
        };
        let drift_ms = match data.get("drift_ms") {
            Some(Value::Array(v)) if v.len() == 2 => Some((number(&v[0]).unwrap_or(0.0), number(&v[1]).unwrap_or(0.0))),
            _ => None,
        };
        Ok(Mission {
            waypoints,
            name: data.get("name").and_then(Value::as_str).unwrap_or("mission").to_string(),
            looped: data.get("loop").and_then(Value::as_bool).unwrap_or(false),
            area: data.get("area").and_then(Rectangle::from_json),
            pattern: data.get("pattern").map(PatternSpec::from_json).unwrap_or_default(),
            drift_ms,
        })
    }

    /// The route as a QGroundControl / ArduPilot `.waypoints` file (QGC WPL
    /// 110): the home first, then a speed change wherever a leg has its own
    /// speed, then each waypoint, holding where it holds.
    pub fn to_waypoints_file(&self, home: Option<LatLon>, cruise_ms: f64) -> String {
        let mut lines = vec!["QGC WPL 110".to_string()];
        let home = home.or_else(|| self.waypoints.first().map(Waypoint::position)).unwrap_or_default();
        lines.push(format!("0\t1\t0\t16\t0\t0\t0\t0\t{:.8}\t{:.8}\t0\t1", home.lat, home.lon));
        let mut index = 1;
        let mut speed = cruise_ms;
        for w in &self.waypoints {
            let wanted = w.speed_ms.unwrap_or(cruise_ms);
            if (wanted - speed).abs() > 1e-6 {
                // MAV_CMD_DO_CHANGE_SPEED: ground speed, m/s, no throttle change.
                lines.push(format!("{index}\t0\t3\t178\t1\t{wanted:.2}\t-1\t0\t0\t0\t0\t1"));
                index += 1;
                speed = wanted;
            }
            // MAV_CMD_NAV_WAYPOINT: hold time, then the position.
            lines.push(format!("{index}\t0\t3\t16\t{:.1}\t0\t0\t0\t{:.8}\t{:.8}\t0\t1", w.hold_s, w.lat, w.lon));
            index += 1;
        }
        lines.join("\n") + "\n"
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        std::fs::write(path, serde_json::to_string_pretty(&self.to_json()).unwrap_or_default())
    }

    pub fn load(path: &Path) -> Result<Mission, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let value: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        Mission::from_json(&value)
    }
}

/// Where `position` is relative to the leg `start` → `end`.
///
/// Returns `(along, across, leg_length)` in metres.  `across` is positive to
/// starboard of the leg, the sign convention of an XTE display: positive
/// means steer to port.
pub fn cross_track(start: LatLon, end: LatLon, position: LatLon) -> (f64, f64, f64) {
    let leg = geo::to_metres(start, end);
    let here = geo::to_metres(start, position);
    let length = leg.east.hypot(leg.north);
    if length < 1e-6 {
        return (0.0, 0.0, 0.0);
    }
    let (ue, un) = (leg.east / length, leg.north / length);
    let along = here.east * ue + here.north * un;
    let across = here.east * un - here.north * ue;
    (along, across, length)
}
