//! Drawing in view space.
//!
//! Every shape on the chart is worked out in view space and handed to these
//! helpers, which take it to the screen through the [`Camera`]: unchanged in
//! the flat views, through the perspective in the chase view (clipped first to
//! the ground image, so nothing behind the camera is drawn).

use egui::epaint::{Mesh, PathStroke, Vertex};
use egui::{Color32, Painter, Pos2, Rect, Shape, Stroke, TextureId};

use super::projection::{clip_segment, Camera};

type V = [f64; 2];

/// The rectangle view-space shapes are clipped to in the chase view.
fn ground_box(cam: &Camera) -> Option<(V, V)> {
    let c = cam.chase.as_ref()?;
    let g = &c.ground;
    let margin = 0.5 * g.w.max(g.h);
    Some(([-margin, -0.08 * g.h], [g.w + margin, g.h + margin]))
}

pub fn line(p: &Painter, cam: &Camera, a: V, b: V, stroke: Stroke) {
    let (a, b) = match ground_box(cam) {
        None => (a, b),
        Some((min, max)) => match clip_segment(a, b, min, max) {
            Some(s) => s,
            None => return,
        },
    };
    let (Some(sa), Some(sb)) = (cam.screen(a), cam.screen(b)) else { return };
    p.line_segment([sa, sb], stroke);
}

pub fn polyline(p: &Painter, cam: &Camera, points: &[V], stroke: Stroke) {
    if points.len() < 2 {
        return;
    }
    if !cam.is_chase() {
        p.add(Shape::line(points.iter().map(|v| Pos2::new(v[0] as f32, v[1] as f32)).collect(), PathStroke::new(stroke.width, stroke.color)));
        return;
    }
    // In the chase view, runs of visible points, cut where they leave the ground.
    let (min, max) = ground_box(cam).unwrap();
    let mut run: Vec<Pos2> = Vec::new();
    let flush = |run: &mut Vec<Pos2>| {
        if run.len() >= 2 {
            p.add(Shape::line(std::mem::take(run), PathStroke::new(stroke.width, stroke.color)));
        }
        run.clear();
    };
    for w in points.windows(2) {
        match clip_segment(w[0], w[1], min, max) {
            Some((a, b)) => {
                let (Some(sa), Some(sb)) = (cam.screen(a), cam.screen(b)) else {
                    flush(&mut run);
                    continue;
                };
                if run.is_empty() || a != w[0] {
                    flush(&mut run);
                    run.push(sa);
                }
                run.push(sb);
                if b != w[1] {
                    flush(&mut run);
                }
            }
            None => flush(&mut run),
        }
    }
    flush(&mut run);
}

pub fn dashed(p: &Painter, cam: &Camera, a: V, b: V, stroke: Stroke, dash: f32, gap: f32) {
    let (a, b) = match ground_box(cam) {
        None => (a, b),
        Some((min, max)) => match clip_segment(a, b, min, max) {
            Some(s) => s,
            None => return,
        },
    };
    let (Some(sa), Some(sb)) = (cam.screen(a), cam.screen(b)) else { return };
    p.extend(Shape::dashed_line(&[sa, sb], stroke, dash, gap));
}

/// Clip a polygon to the chase view's ground (Sutherland–Hodgman).
fn clip_polygon(points: &[V], min: V, max: V) -> Vec<V> {
    let mut out: Vec<V> = points.to_vec();
    for (axis, bound, keep_above) in [(0, min[0], true), (0, max[0], false), (1, min[1], true), (1, max[1], false)] {
        let input = std::mem::take(&mut out);
        if input.is_empty() {
            break;
        }
        let inside = |v: &V| if keep_above { v[axis] >= bound } else { v[axis] <= bound };
        for i in 0..input.len() {
            let (cur, prev) = (input[i], input[(i + input.len() - 1) % input.len()]);
            let cross = |a: V, b: V| {
                let t = (bound - a[axis]) / (b[axis] - a[axis]);
                [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
            };
            match (inside(&cur), inside(&prev)) {
                (true, true) => out.push(cur),
                (true, false) => {
                    out.push(cross(prev, cur));
                    out.push(cur);
                }
                (false, true) => out.push(cross(prev, cur)),
                (false, false) => {}
            }
        }
    }
    out
}

/// A convex polygon, filled and/or outlined.
pub fn polygon(p: &Painter, cam: &Camera, points: &[V], fill: Color32, stroke: Stroke) {
    let points = match ground_box(cam) {
        None => points.to_vec(),
        Some((min, max)) => clip_polygon(points, min, max),
    };
    if points.len() < 3 {
        return;
    }
    let screen: Vec<Pos2> = points.iter().map(|v| cam.screen_unclipped(*v)).collect();
    p.add(Shape::convex_polygon(screen, fill, PathStroke::new(stroke.width, stroke.color)));
}

/// A closed outline, convex or not.
pub fn closed(p: &Painter, cam: &Camera, points: &[V], stroke: Stroke) {
    if points.len() < 2 {
        return;
    }
    let mut ring = points.to_vec();
    ring.push(points[0]);
    polyline(p, cam, &ring, stroke);
}

pub fn closed_dashed(p: &Painter, cam: &Camera, points: &[V], stroke: Stroke, dash: f32, gap: f32) {
    for i in 0..points.len() {
        dashed(p, cam, points[i], points[(i + 1) % points.len()], stroke, dash, gap);
    }
}

pub fn circle_points(centre: V, radius: f64, segments: usize) -> Vec<V> {
    (0..segments)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / segments as f64;
            [centre[0] + radius * a.cos(), centre[1] + radius * a.sin()]
        })
        .collect()
}

pub fn circle(p: &Painter, cam: &Camera, centre: V, radius: f64, fill: Color32, stroke: Stroke) {
    if !cam.is_chase() {
        p.circle(Pos2::new(centre[0] as f32, centre[1] as f32), radius as f32, fill, stroke);
        return;
    }
    let pts = circle_points(centre, radius, 24);
    if fill != Color32::TRANSPARENT {
        polygon(p, cam, &pts, fill, Stroke::NONE);
    }
    if stroke.width > 0.0 {
        closed(p, cam, &pts, stroke);
    }
}

pub fn rect_points(centre: V, half: f64) -> Vec<V> {
    vec![
        [centre[0] - half, centre[1] - half],
        [centre[0] + half, centre[1] - half],
        [centre[0] + half, centre[1] + half],
        [centre[0] - half, centre[1] + half],
    ]
}

/// An arrow head at `at` of the way along `a`→`b`, as the route arrows are.
pub fn arrow(p: &Painter, cam: &Camera, a: V, b: V, colour: Color32, at: f64, size: f64, min_length: f64) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    if dx.hypot(dy) < min_length {
        return;
    }
    let mid = [a[0] + dx * at, a[1] + dy * at];
    let angle = dy.atan2(dx);
    let pt = |off: f64| [mid[0] + (angle + off).cos() * size, mid[1] + (angle + off).sin() * size];
    polygon(p, cam, &[pt(0.0), pt(2.5), pt(-2.5)], colour, Stroke::NONE);
}

/// A texture laid on the water between four geographic edges.
///
/// `edges` is north, west, south, east; `uv` the part of the texture they
/// span.  The part outside the camera's view is cut off first (so a deep
/// zoom does not stretch a whole gulf to draw a few cells).  In the chase
/// view the picture is cut into a grid of small pieces, each laid flat, which
/// is close enough to a true perspective.
pub fn geo_image(p: &Painter, cam: &Camera, texture: TextureId, edges: [f64; 4], uv: Rect, tint: Color32) {
    let [north, west, south, east] = edges;
    if !(north > south && east > west) {
        return;
    }
    let (bs, bw, bn, be) = cam.bounds();
    let (n, w, s, e) = (north.min(bn), west.max(bw), south.max(bs), east.min(be));
    if n <= s || e <= w {
        return;
    }
    let u = |lon: f64| uv.min.x + (((lon - west) / (east - west)) as f32) * uv.width();
    let v = |lat: f64| uv.min.y + (((north - lat) / (north - south)) as f32) * uv.height();
    let mut mesh = Mesh::with_texture(texture);
    let (nx, ny) = if cam.is_chase() { (28usize, 28usize) } else { (1, 1) };
    let mut ok = Vec::with_capacity((nx + 1) * (ny + 1));
    for j in 0..=ny {
        let lat = n - (n - s) * j as f64 / ny as f64;
        for i in 0..=nx {
            let lon = w + (e - w) * i as f64 / nx as f64;
            let view = cam.view(crate::core::geo::LatLon::new(lat, lon));
            ok.push(cam.visible(view));
            mesh.vertices.push(Vertex { pos: cam.screen_unclipped(view), uv: Pos2::new(u(lon), v(lat)), color: tint });
        }
    }
    let idx = |i: usize, j: usize| (j * (nx + 1) + i) as u32;
    for j in 0..ny {
        for i in 0..nx {
            let corners = [idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)];
            if corners.iter().any(|&c| !ok[c as usize]) {
                continue;
            }
            mesh.add_triangle(corners[0], corners[1], corners[2]);
            mesh.add_triangle(corners[0], corners[2], corners[3]);
        }
    }
    if !mesh.indices.is_empty() {
        p.add(Shape::mesh(mesh));
    }
}

/// A rectangle filled with a vertical gradient through `stops` (0..1, colour).
pub fn vertical_gradient(p: &Painter, rect: Rect, stops: &[(f32, Color32)]) {
    let mut mesh = Mesh::default();
    for w in stops.windows(2) {
        let (t0, c0) = w[0];
        let (t1, c1) = w[1];
        let y0 = rect.top() + rect.height() * t0;
        let y1 = rect.top() + rect.height() * t1;
        let base = mesh.vertices.len() as u32;
        mesh.colored_vertex(Pos2::new(rect.left(), y0), c0);
        mesh.colored_vertex(Pos2::new(rect.right(), y0), c0);
        mesh.colored_vertex(Pos2::new(rect.right(), y1), c1);
        mesh.colored_vertex(Pos2::new(rect.left(), y1), c1);
        mesh.add_triangle(base, base + 1, base + 2);
        mesh.add_triangle(base, base + 2, base + 3);
    }
    p.add(Shape::mesh(mesh));
}

/// An ellipse filled with a radial gradient through `stops` (0..1 of the radius).
pub fn radial_gradient(p: &Painter, centre: Pos2, radius: egui::Vec2, stops: &[(f32, Color32)]) {
    const SEGMENTS: usize = 48;
    let mut mesh = Mesh::default();
    let ring = |mesh: &mut Mesh, t: f32, colour: Color32| {
        let base = mesh.vertices.len() as u32;
        for i in 0..SEGMENTS {
            let a = std::f32::consts::TAU * i as f32 / SEGMENTS as f32;
            mesh.colored_vertex(Pos2::new(centre.x + radius.x * t * a.cos(), centre.y + radius.y * t * a.sin()), colour);
        }
        base
    };
    let mut previous: Option<u32> = None;
    for &(t, colour) in stops {
        let base = ring(&mut mesh, t.max(1e-3), colour);
        if let Some(prev) = previous {
            for i in 0..SEGMENTS as u32 {
                let j = (i + 1) % SEGMENTS as u32;
                mesh.add_triangle(prev + i, prev + j, base + j);
                mesh.add_triangle(prev + i, base + j, base + i);
            }
        } else {
            let c = mesh.vertices.len() as u32;
            mesh.colored_vertex(centre, colour);
            for i in 0..SEGMENTS as u32 {
                mesh.add_triangle(c, base + i, base + (i + 1) % SEGMENTS as u32);
            }
        }
        previous = Some(base);
    }
    p.add(Shape::mesh(mesh));
}

/// Points round a rounded rectangle, for shapes that have to be turned.
pub fn rounded_rect_points(rect: Rect, r: f32) -> Vec<Pos2> {
    let r = r.min(rect.width() / 2.0).min(rect.height() / 2.0);
    let mut pts = Vec::with_capacity(20);
    let corners = [
        (Pos2::new(rect.right() - r, rect.top() + r), -90.0f32),
        (Pos2::new(rect.right() - r, rect.bottom() - r), 0.0),
        (Pos2::new(rect.left() + r, rect.bottom() - r), 90.0),
        (Pos2::new(rect.left() + r, rect.top() + r), 180.0),
    ];
    for (c, start) in corners {
        for k in 0..=4 {
            let a = (start + 90.0 * k as f32 / 4.0).to_radians();
            pts.push(Pos2::new(c.x + r * a.cos(), c.y + r * a.sin()));
        }
    }
    pts
}
