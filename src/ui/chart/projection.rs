//! Where things are on the screen.
//!
//! Coordinates come in three spaces.  *Map space* is north-up pixels.  *View
//! space* is map space turned to the view's rotation about its middle; it is
//! the screen itself in the flat views.  In the chase view, view space is a
//! "ground image" a little wider and much taller than the screen, laid onto
//! the screen by a perspective transform (a homography), the way a driving
//! game shows the road ahead.  Straight lines stay straight under it, so
//! shapes are worked out in view space and only their corners transformed.

use egui::{Pos2, Rect};

use crate::core::geo::{self, LatLon};

/// Flat-earth projection about a centre, at `scale` pixels per degree of
/// latitude, turned by `rotation` degrees (the bearing that points up).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Projection {
    /// Top-left of the space, and its size, pixels.
    pub ox: f64,
    pub oy: f64,
    pub w: f64,
    pub h: f64,
    pub centre: LatLon,
    pub scale: f64,
    pub rotation: f64,
}

impl Projection {
    pub fn lon_scale(&self) -> f64 {
        self.centre.lat.to_radians().cos().max(0.05)
    }

    fn mid(&self) -> (f64, f64) {
        (self.ox + self.w / 2.0, self.oy + self.h / 2.0)
    }

    /// Map space: north-up pixels.
    pub fn to_map(&self, p: LatLon) -> [f64; 2] {
        let (mx, my) = self.mid();
        [mx + (p.lon - self.centre.lon) * self.scale * self.lon_scale(), my - (p.lat - self.centre.lat) * self.scale]
    }

    pub fn from_map(&self, x: f64, y: f64) -> LatLon {
        let (mx, my) = self.mid();
        LatLon::new(
            self.centre.lat - (y - my) / self.scale,
            self.centre.lon + (x - mx) / (self.scale * self.lon_scale()),
        )
    }

    /// Map space to view space: a turn of `-rotation` about the middle.
    pub fn rotate(&self, p: [f64; 2]) -> [f64; 2] {
        if self.rotation == 0.0 {
            return p;
        }
        let (mx, my) = self.mid();
        let (s, c) = (-self.rotation).to_radians().sin_cos();
        let (dx, dy) = (p[0] - mx, p[1] - my);
        [mx + dx * c - dy * s, my + dx * s + dy * c]
    }

    pub fn unrotate(&self, p: [f64; 2]) -> [f64; 2] {
        if self.rotation == 0.0 {
            return p;
        }
        let (mx, my) = self.mid();
        let (s, c) = self.rotation.to_radians().sin_cos();
        let (dx, dy) = (p[0] - mx, p[1] - my);
        [mx + dx * c - dy * s, my + dx * s + dy * c]
    }

    pub fn to_view(&self, p: LatLon) -> [f64; 2] {
        self.rotate(self.to_map(p))
    }

    pub fn from_view(&self, p: [f64; 2]) -> LatLon {
        let m = self.unrotate(p);
        self.from_map(m[0], m[1])
    }

    pub fn metres_per_pixel(&self) -> f64 {
        geo::metres_per_degree(self.centre.lat).0 / self.scale
    }

    /// South, west, north, east of everything in the space, however it turns.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let corners = [
            [self.ox, self.oy],
            [self.ox + self.w, self.oy],
            [self.ox, self.oy + self.h],
            [self.ox + self.w, self.oy + self.h],
        ];
        let points: Vec<LatLon> = corners.iter().map(|c| self.from_view(*c)).collect();
        let lats = points.iter().map(|p| p.lat);
        let lons = points.iter().map(|p| p.lon);
        (
            lats.clone().fold(f64::INFINITY, f64::min),
            lons.clone().fold(f64::INFINITY, f64::min),
            lats.fold(f64::NEG_INFINITY, f64::max),
            lons.fold(f64::NEG_INFINITY, f64::max),
        )
    }

    /// A unit vector in view space for a compass bearing at `origin`.
    pub fn direction(&self, origin: LatLon, bearing: f64) -> [f64; 2] {
        let a = self.to_view(origin);
        let b = self.to_view(geo::destination(origin, bearing, 10.0));
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy).max(1e-9);
        [dx / len, dy / len]
    }
}

/// A plane-to-plane perspective transform, as a 3×3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Homography {
    pub m: [[f64; 3]; 3],
}

impl Homography {
    /// The transform taking the unit square's corners (0,0) (1,0) (1,1) (0,1)
    /// to `q` (Heckbert's square-to-quad).
    pub fn square_to_quad(q: [[f64; 2]; 4]) -> Homography {
        let [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] = q;
        let (dx1, dx2, dx3) = (x1 - x2, x3 - x2, x0 - x1 + x2 - x3);
        let (dy1, dy2, dy3) = (y1 - y2, y3 - y2, y0 - y1 + y2 - y3);
        let (a, b, c, d, e, f, g, h);
        if dx3.abs() < 1e-12 && dy3.abs() < 1e-12 {
            a = x1 - x0;
            b = x2 - x1;
            c = x0;
            d = y1 - y0;
            e = y2 - y1;
            f = y0;
            g = 0.0;
            h = 0.0;
        } else {
            let det = dx1 * dy2 - dx2 * dy1;
            g = (dx3 * dy2 - dx2 * dy3) / det;
            h = (dx1 * dy3 - dx3 * dy1) / det;
            a = x1 - x0 + g * x1;
            b = x3 - x0 + h * x3;
            c = x0;
            d = y1 - y0 + g * y1;
            e = y3 - y0 + h * y3;
            f = y0;
        }
        Homography { m: [[a, b, c], [d, e, f], [g, h, 1.0]] }
    }

    /// The rectangle (0,0)–(w,h) onto the quad `q`.
    pub fn rect_to_quad(w: f64, h: f64, q: [[f64; 2]; 4]) -> Homography {
        let s = Homography::square_to_quad(q);
        s.compose(&Homography { m: [[1.0 / w, 0.0, 0.0], [0.0, 1.0 / h, 0.0], [0.0, 0.0, 1.0]] })
    }

    /// `self` after `other`.
    pub fn compose(&self, other: &Homography) -> Homography {
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| self.m[i][k] * other.m[k][j]).sum();
            }
        }
        Homography { m }
    }

    pub fn inverse(&self) -> Option<Homography> {
        let m = &self.m;
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if det.abs() < 1e-15 {
            return None;
        }
        let inv = |a: usize, b: usize, c: usize, d: usize| (m[a][b] * m[c][d] - m[a][d] * m[c][b]) / det;
        Some(Homography {
            m: [
                [inv(1, 1, 2, 2), -inv(0, 1, 2, 2), inv(0, 1, 1, 2)],
                [-inv(1, 0, 2, 2), inv(0, 0, 2, 2), -inv(0, 0, 1, 2)],
                [inv(1, 0, 2, 1), -inv(0, 0, 2, 1), inv(0, 0, 1, 1)],
            ],
        })
    }

    /// The image of a point, and the projective denominator (≤ 0 means the
    /// point is behind the camera).
    pub fn apply(&self, p: [f64; 2]) -> ([f64; 2], f64) {
        let m = &self.m;
        let w = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2];
        let x = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2];
        let y = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2];
        ([x / w, y / w], w)
    }
}

/// The chase camera's screen geometry.
pub const HORIZON: f64 = 0.20;
/// How much bigger than the screen the ground image is (it is tall, to reach far ahead).
pub const CHASE_W: f64 = 1.3;
pub const CHASE_H: f64 = 2.2;
/// The ground image works at this fraction of screen resolution.
pub const CHASE_RES: f64 = 0.5;
/// How far ahead of the boat the ground image is centred, as a fraction of
/// its height; this puts the boat at about 60 % of the screen height.
pub const CHASE_AHEAD: f64 = 0.17;

#[derive(Clone, Copy, Debug)]
pub struct Chase {
    pub ground: Projection,
    pub to_screen: Homography,
    pub from_screen: Homography,
    pub horizon_y: f64,
}

impl Chase {
    /// The ground plane for a screen `rect`, its middle at `centre`.
    pub fn new(rect: Rect, centre: LatLon, scale: f64, rotation: f64) -> Option<Chase> {
        let (w, h) = (rect.width() as f64, rect.height() as f64);
        let (gw, gh) = ((w * CHASE_W * CHASE_RES).floor().max(1.0), (h * CHASE_H * CHASE_RES).floor().max(1.0));
        let ground = Projection { ox: 0.0, oy: 0.0, w: gw, h: gh, centre, scale: scale * CHASE_RES, rotation };
        let (x0, y0) = (rect.min.x as f64, rect.min.y as f64);
        let horizon = y0 + h * HORIZON;
        let quad = [
            [x0 + w * 0.5 - w * 0.7, horizon],
            [x0 + w * 0.5 + w * 0.7, horizon],
            [x0 + w * 0.5 + w * 2.2, y0 + h * 1.25],
            [x0 + w * 0.5 - w * 2.2, y0 + h * 1.25],
        ];
        let to_screen = Homography::rect_to_quad(gw, gh, quad);
        let from_screen = to_screen.inverse()?;
        Some(Chase { ground, to_screen, from_screen, horizon_y: horizon })
    }

    pub fn ground_size(rect: Rect) -> (f64, f64) {
        (
            (rect.width() as f64 * CHASE_W * CHASE_RES).floor(),
            (rect.height() as f64 * CHASE_H * CHASE_RES).floor(),
        )
    }
}

/// How the chart reaches the screen: a flat projection, or the chase camera.
#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub rect: Rect,
    /// View space.  In the flat views this is the screen.
    pub view: Projection,
    pub chase: Option<Chase>,
}

impl Camera {
    pub fn flat(rect: Rect, centre: LatLon, scale: f64, rotation: f64) -> Camera {
        let view = Projection {
            ox: rect.min.x as f64,
            oy: rect.min.y as f64,
            w: rect.width() as f64,
            h: rect.height() as f64,
            centre,
            scale,
            rotation,
        };
        Camera { rect, view, chase: None }
    }

    pub fn chase(rect: Rect, centre: LatLon, scale: f64, rotation: f64) -> Option<Camera> {
        let chase = Chase::new(rect, centre, scale, rotation)?;
        Some(Camera { rect, view: chase.ground, chase: Some(chase) })
    }

    pub fn is_chase(&self) -> bool {
        self.chase.is_some()
    }

    /// View space position of a place.
    pub fn view(&self, p: LatLon) -> [f64; 2] {
        self.view.to_view(p)
    }

    /// Whether a view-space point can be shown: in the chase view, only the
    /// ground image and a little beyond it, never behind the camera.
    pub fn visible(&self, v: [f64; 2]) -> bool {
        match &self.chase {
            None => true,
            Some(c) => {
                let g = &c.ground;
                let margin = 0.5 * g.w.max(g.h);
                v[1] >= -0.08 * g.h && v[1] <= g.h + margin && v[0] >= -margin && v[0] <= g.w + margin
            }
        }
    }

    /// A view-space point on the screen, None if it cannot be shown.
    pub fn screen(&self, v: [f64; 2]) -> Option<Pos2> {
        match &self.chase {
            None => Some(Pos2::new(v[0] as f32, v[1] as f32)),
            Some(c) => {
                if !self.visible(v) {
                    return None;
                }
                let (s, w) = c.to_screen.apply(v);
                (w > 1e-6).then(|| Pos2::new(s[0] as f32, s[1] as f32))
            }
        }
    }

    /// A view-space point on the screen, however far off it is.
    pub fn screen_unclipped(&self, v: [f64; 2]) -> Pos2 {
        match &self.chase {
            None => Pos2::new(v[0] as f32, v[1] as f32),
            Some(c) => {
                let (s, _) = c.to_screen.apply(v);
                Pos2::new(s[0] as f32, s[1] as f32)
            }
        }
    }

    pub fn project(&self, p: LatLon) -> Option<Pos2> {
        self.screen(self.view(p))
    }

    /// The water under a point on the screen.
    pub fn unproject(&self, pos: Pos2) -> Option<LatLon> {
        match &self.chase {
            None => Some(self.view.from_view([pos.x as f64, pos.y as f64])),
            Some(c) => {
                if (pos.y as f64) < c.horizon_y + 2.0 {
                    return None;
                }
                let (v, w) = c.from_screen.apply([pos.x as f64, pos.y as f64]);
                (w.abs() > 1e-9).then(|| self.view.from_view(v))
            }
        }
    }

    /// How many screen pixels one view pixel spans at `v` (1 in the flat views).
    pub fn scale_at(&self, v: [f64; 2]) -> f32 {
        match &self.chase {
            None => 1.0,
            Some(c) => {
                let (a, _) = c.to_screen.apply(v);
                let (b, _) = c.to_screen.apply([v[0] + 1.0, v[1]]);
                let (d, _) = c.to_screen.apply([v[0], v[1] + 1.0]);
                let cross = ((b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0])).abs();
                cross.sqrt() as f32
            }
        }
    }

    pub fn metres_per_pixel(&self) -> f64 {
        self.view.metres_per_pixel()
    }

    /// South, west, north, east of the water that can appear.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        self.view.bounds()
    }
}

/// Clip the segment `a`–`b` to a rectangle (Liang–Barsky).
pub fn clip_segment(a: [f64; 2], b: [f64; 2], min: [f64; 2], max: [f64; 2]) -> Option<([f64; 2], [f64; 2])> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (p, q) in [(-dx, a[0] - min[0]), (dx, max[0] - a[0]), (-dy, a[1] - min[1]), (dy, max[1] - a[1])] {
        if p == 0.0 {
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return None;
                }
                t0 = t0.max(r);
            } else {
                if r < t0 {
                    return None;
                }
                t1 = t1.min(r);
            }
        }
    }
    Some(([a[0] + t0 * dx, a[1] + t0 * dy], [a[0] + t1 * dx, a[1] + t1 * dy]))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SEA: LatLon = LatLon::new(57.10, 23.95);

    fn rect() -> Rect {
        Rect::from_min_size(Pos2::new(100.0, 40.0), egui::vec2(640.0, 640.0))
    }

    #[test]
    fn taps_land_where_they_are_drawn_north_up() {
        let cam = Camera::flat(rect(), SEA, 180_000.0, 0.0);
        let target = geo::destination(SEA, 70.0, 60.0);
        let at = cam.project(target).unwrap();
        assert!(geo::distance_m(cam.unproject(at).unwrap(), target) < 1.0);
    }

    #[test]
    fn heading_up_puts_the_bow_up() {
        let cam = Camera::flat(rect(), SEA, 180_000.0, 50.0);
        let target = geo::destination(SEA, 70.0, 60.0);
        let at = cam.project(target).unwrap();
        assert!(geo::distance_m(cam.unproject(at).unwrap(), target) < 1.0);
        let here = cam.project(SEA).unwrap();
        let ahead = cam.project(geo::destination(SEA, 50.0, 50.0)).unwrap();
        assert!((ahead.x - here.x).abs() < 3.0 && ahead.y < here.y);
    }

    #[test]
    fn chase_taps_land_where_they_are_drawn() {
        let cam = Camera::chase(rect(), geo::destination(SEA, 50.0, 40.0), 180_000.0, 50.0).unwrap();
        let target = geo::destination(SEA, 70.0, 60.0);
        let at = cam.project(target).unwrap();
        assert!(at.y as f64 > cam.chase.unwrap().horizon_y);
        assert!(geo::distance_m(cam.unproject(at).unwrap(), target) < 1.0);
        // Farther away is higher up the screen, and nearer the horizon.
        let far = cam.project(geo::destination(SEA, 50.0, 300.0)).unwrap();
        let near = cam.project(SEA).unwrap();
        assert!(far.y < near.y);
        assert!(cam.unproject(Pos2::new(300.0, 41.0)).is_none(), "the sky is not water");
    }

    #[test]
    fn homography_round_trips() {
        let h = Homography::rect_to_quad(
            100.0,
            50.0,
            [[10.0, 10.0], [90.0, 10.0], [140.0, 80.0], [-40.0, 80.0]],
        );
        let inv = h.inverse().unwrap();
        for p in [[0.0, 0.0], [100.0, 0.0], [50.0, 25.0], [100.0, 50.0]] {
            let (s, _) = h.apply(p);
            let (back, _) = inv.apply(s);
            assert!((back[0] - p[0]).abs() < 1e-6 && (back[1] - p[1]).abs() < 1e-6);
        }
        let (corner, _) = h.apply([100.0, 50.0]);
        assert!((corner[0] - 140.0).abs() < 1e-6 && (corner[1] - 80.0).abs() < 1e-6);
    }

    #[test]
    fn segments_clip_to_the_box() {
        let (a, b) = clip_segment([-10.0, 5.0], [20.0, 5.0], [0.0, 0.0], [10.0, 10.0]).unwrap();
        assert_eq!(a, [0.0, 5.0]);
        assert_eq!(b, [10.0, 5.0]);
        assert!(clip_segment([-10.0, -5.0], [-1.0, -5.0], [0.0, 0.0], [10.0, 10.0]).is_none());
    }
}
