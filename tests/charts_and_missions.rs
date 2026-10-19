mod common;

use common::chart;
use usvgs::core::bathymetry::{contour_segments, DepthGrid, SoundingGrid, ESTIMATED};
use usvgs::core::geo::{self, LatLon};
use usvgs::core::mission::{cross_track, lawnmower, perimeter, Mission, PatternSpec, Rectangle};

const CENTRE: LatLon = LatLon::new(57.05, 23.99);

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

// -- missions ---------------------------------------------------------------------

#[test]
fn rectangle_corners_are_its_size() {
    let c = Rectangle::new(CENTRE, 200.0, 80.0, 90.0).corners();
    assert!(close(geo::distance_m(c[0], c[1]), 200.0, 0.1));
    assert!(close(geo::distance_m(c[1], c[2]), 80.0, 0.1));
    // Width axis runs east at 90°.
    assert!(close(geo::bearing_deg(c[0], c[1]), 90.0, 0.1));
}

#[test]
fn rectangle_from_corners_round_trips_when_rotated() {
    let c = Rectangle::new(CENTRE, 150.0, 60.0, 35.0).corners();
    let again = Rectangle::from_corners(c[0], c[2], 35.0);
    assert!(close(again.width_m, 150.0, 0.2));
    assert!(close(again.height_m, 60.0, 0.2));
    assert!(geo::distance_m(again.centre, CENTRE) < 0.2);
}

#[test]
fn lawnmower_covers_the_rectangle_with_alternating_lanes() {
    let rect = Rectangle::new(CENTRE, 100.0, 40.0, 90.0);
    let route = lawnmower(&rect, 10.0, 0.0, true);
    assert_eq!(route.len(), 2 * 5); // 40 m across at 10 m: five lanes
    let first = geo::bearing_deg(route[0], route[1]);
    let second = geo::bearing_deg(route[2], route[3]);
    assert!(close(geo::angle_difference(first, second).abs(), 180.0, 0.5));
    for lane in route.chunks(2) {
        assert!(close(geo::distance_m(lane[0], lane[1]), 100.0, 0.2));
    }
}

#[test]
fn lawnmower_runs_along_the_short_side_when_asked() {
    let rect = Rectangle::new(CENTRE, 100.0, 40.0, 90.0);
    let route = lawnmower(&rect, 10.0, 0.0, false);
    assert!(close(geo::distance_m(route[0], route[1]), 40.0, 0.2));
}

#[test]
fn perimeter_closes() {
    let route = perimeter(&Rectangle::new(CENTRE, 50.0, 30.0, 10.0), 0.0, 2);
    assert_eq!(route.len(), 9);
    assert!(geo::distance_m(route[0], route[8]) < 0.01);
    assert!(close(geo::path_length_m(&route), 320.0, 0.5));
}

#[test]
fn mission_file_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let mut mission = Mission { name: "test".into(), looped: true, ..Mission::default() };
    mission.area = Some(Rectangle::new(CENTRE, 80.0, 40.0, 45.0));
    mission.pattern = PatternSpec { spacing_m: 8.0, ..PatternSpec::default() };
    assert!(mission.regenerate());
    mission.waypoints[0].hold_s = 5.0;
    let path = dir.path().join("m.json");
    mission.save(&path).unwrap();
    let loaded = Mission::load(&path).unwrap();
    assert!(loaded.looped && loaded.name == "test");
    assert_eq!(loaded.waypoints.len(), mission.waypoints.len());
    assert_eq!(loaded.waypoints[0].hold_s, 5.0);
    assert_eq!(loaded.area.unwrap().angle_deg, 45.0);
    assert_eq!(loaded.pattern.spacing_m, 8.0);
}

#[test]
fn python_mission_files_still_open() {
    let text = r#"{"format": "usvgs-mission", "version": 1, "name": "old", "loop": false,
        "area": null, "pattern": {"pattern": "perimeter", "spacing_m": 10.0, "margin_m": 2.0,
        "along_long_side": true, "laps": 3}, "drift_ms": [0.1, 0.2],
        "waypoints": [{"lat": 57.05, "lon": 23.99}, {"lat": 57.051, "lon": 23.991, "speed_ms": 0.5}]}"#;
    let m = Mission::from_json(&serde_json::from_str(text).unwrap()).unwrap();
    assert_eq!(m.waypoints.len(), 2);
    assert_eq!(m.waypoints[1].speed_ms, Some(0.5));
    assert_eq!(m.pattern.laps, 3);
    assert_eq!(m.drift_ms, Some((0.1, 0.2)));
}

#[test]
fn cross_track_sign_is_positive_to_starboard() {
    let end = geo::destination(CENTRE, 0.0, 100.0);
    let east_of_line = geo::destination(geo::destination(CENTRE, 0.0, 50.0), 90.0, 5.0);
    let (along, across, length) = cross_track(CENTRE, end, east_of_line);
    assert!(close(along, 50.0, 0.1));
    assert!(close(across, 5.0, 0.1));
    assert!(close(length, 100.0, 0.1));
}

// -- bathymetry -------------------------------------------------------------------

#[test]
fn bundled_chart_knows_sea_from_land() {
    let chart = chart();
    assert!(chart.depth_at(57.10, 23.95) > 10.0); // Gulf of Riga
    assert!(chart.depth_at(56.95, 24.12).is_nan()); // Old Riga is not water
    assert_eq!(chart.quality_at(57.02, 24.18), ESTIMATED); // Ķīšezers is estimated
    assert_eq!((chart.rows, chart.cols), (528, 1152));
    assert!(chart.source.starts_with("EMODnet"));
}

#[test]
fn npz_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let grid = DepthGrid::new(vec![1.0, 2.0, f32::NAN, 4.5], 2, 2, 57.0, 24.0, 0.001, 0.002).with_source("t");
    let path = dir.path().join("g.npz");
    grid.save_npz(&path).unwrap();
    let again = DepthGrid::load(&path).unwrap();
    assert!(again.at(1, 0).is_nan());
    assert!(close(again.at(1, 1) as f64, 4.5, 1e-6));
    assert!(close(again.dlon, 0.002, 1e-12));
    assert_eq!(again.source, "t");
}

#[test]
fn interpolation_is_bilinear_in_open_water() {
    let grid = DepthGrid::new(vec![2.0, 4.0, 2.0, 4.0], 2, 2, 1.0, 0.0, 0.5, 0.5);
    assert!(close(grid.depth_at(0.5, 0.5), 3.0, 1e-9));
}

#[test]
fn ascii_grid_is_read_as_elevation() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("g.asc");
    std::fs::write(
        &path,
        "ncols 2\nnrows 2\nxllcorner 24.0\nyllcorner 57.0\ncellsize 0.01\nNODATA_value -9999\n-3 -5\n2 -9999\n",
    )
    .unwrap();
    let grid = DepthGrid::load(&path).unwrap();
    assert!(close(grid.at(0, 0) as f64, 3.0, 1e-6));
    assert!(grid.at(1, 0).is_nan() && grid.at(1, 1).is_nan());
    assert!(close(grid.north, 57.02, 1e-9));
}

#[test]
fn xyz_soundings_are_binned() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("s.csv");
    std::fs::write(&path, "lon,lat,depth\n24.0,57.0,3.0\n24.0001,57.0001,5.0\n24.01,57.01,9.0\n").unwrap();
    let grid = DepthGrid::load(&path).unwrap();
    let deepest = grid.depth.iter().filter(|d| !d.is_nan()).fold(0.0f32, |a, &b| a.max(b));
    assert!(close(deepest as f64, 9.0, 1e-6));
}

#[test]
fn contours_follow_the_level() {
    let depth: Vec<f32> = (0..5).flat_map(|_| (0..10).map(|c| c as f32)).collect();
    let grid = DepthGrid::new(depth, 5, 10, 1.0, 0.0, 0.1, 0.1);
    let segments = contour_segments(&grid, 4.5);
    assert_eq!(segments.len(), 4);
    // Every crossing sits half-way between the columns of depth 4 and 5.
    let expected_lon = grid.west + (4.5 + 0.5) * grid.dlon;
    for s in &segments {
        assert!(close(s[0], expected_lon, 1e-9) && close(s[2], expected_lon, 1e-9));
    }
}

#[test]
fn soundings_fill_cells_and_coverage() {
    let origin = LatLon::new(57.0, 24.0);
    let mut grid = SoundingGrid::new(origin, 5.0);
    let rect = Rectangle::new(origin, 20.0, 20.0, 90.0);
    assert_eq!(grid.coverage(Some(&rect)), 0.0);
    let new = grid.add(57.0, 24.0, 3.2, 10.0, None);
    assert!(new > 1);
    assert!(close(grid.depth_of(grid.cell_of(57.0, 24.0)).unwrap(), 3.2, 1e-9));
    let c = grid.coverage(Some(&rect));
    assert!(0.0 < c && c <= 1.0);
    grid.add(57.0, 24.0, 3.6, 0.0, None);
    assert!(close(grid.depth_of(grid.cell_of(57.0, 24.0)).unwrap(), 3.4, 1e-9));
}

#[test]
fn soundings_export() {
    let dir = tempfile::tempdir().unwrap();
    let mut grid = SoundingGrid::new(LatLon::new(57.0, 24.0), 5.0);
    grid.add(57.0, 24.0, 2.0, 0.0, None);
    let path = dir.path().join("x.csv");
    assert_eq!(grid.export_xyz(&path).unwrap(), 1);
    let reread = DepthGrid::load_xyz(&path, 0.0005).unwrap();
    let deepest = reread.depth.iter().filter(|d| !d.is_nan()).fold(0.0f32, |a, &b| a.max(b));
    assert!(close(deepest as f64, 2.0, 1e-6));
}

// -- the pattern library --------------------------------------------------------------

use usvgs::core::mission::{generate, plan_auto, spacing_for, spiral_in, spiral_out, AutoContext, Pattern};

#[test]
fn overlap_narrows_the_lanes() {
    assert!(close(spacing_for(10.0, 0.0), 10.0, 1e-9));
    assert!(close(spacing_for(10.0, 30.0), 7.0, 1e-9));
}

#[test]
fn spiral_in_ends_in_the_middle_and_out_starts_there() {
    let rect = Rectangle::new(CENTRE, 100.0, 60.0, 90.0);
    let inward = spiral_in(&rect, 10.0, 0.0);
    assert!(geo::distance_m(inward[0], rect.corners()[0]) < 0.5, "starts at a corner");
    let end = *inward.last().unwrap();
    assert!(geo::distance_m(end, CENTRE) < 30.0, "ends near the middle");
    // Laps are a lane apart: the second bottom pass is 10 m inside the first.
    assert!(close(geo::distance_m(inward[0], inward[4]), 10.0, 0.3), "laps are a lane apart");
    let outward = spiral_out(&rect, 10.0, 0.0);
    assert_eq!(outward.first(), inward.last());
    // Everything stays inside the rectangle.
    let roomy = Rectangle::new(CENTRE, 100.5, 60.5, 90.0);
    assert!(inward.iter().all(|p| roomy.contains(*p)));
}

#[test]
fn grid_crosses_its_own_lanes() {
    let rect = Rectangle::new(CENTRE, 80.0, 40.0, 90.0);
    let spec = PatternSpec { pattern: Pattern::Grid, spacing_m: 10.0, margin_m: 0.0, ..PatternSpec::default() };
    let route = generate(&rect, &spec);
    let lanes = lawnmower(&rect, 10.0, 0.0, true).len() + lawnmower(&rect, 10.0, 0.0, false).len();
    assert_eq!(route.len(), lanes);
    // The two halves run at right angles.
    let first = geo::bearing_deg(route[0], route[1]);
    let n = lawnmower(&rect, 10.0, 0.0, true).len();
    let second = geo::bearing_deg(route[n], route[n + 1]);
    assert!(close(geo::angle_difference(first, second).abs(), 90.0, 1.0));
}

#[test]
fn start_corner_moves_the_start() {
    let rect = Rectangle::new(CENTRE, 80.0, 40.0, 30.0);
    for corner in 0..4u8 {
        let spec = PatternSpec { start_corner: corner, margin_m: 0.0, ..PatternSpec::default() };
        let route = generate(&rect, &spec);
        assert!(geo::distance_m(route[0], rect.corners()[corner as usize]) < 0.5, "corner {corner}");
    }
}

#[test]
fn the_planner_starts_near_the_boat_and_avoids_shallows() {
    let rect = Rectangle::new(CENTRE, 120.0, 60.0, 90.0);
    let spec = PatternSpec { pattern: Pattern::Auto, margin_m: 0.0, ..PatternSpec::default() };
    // The boat waits off the north-east corner.
    let boat = geo::destination(rect.corners()[2], 45.0, 200.0);
    let ctx = AutoContext { start: Some(boat), ..AutoContext::default() };
    let plan = plan_auto(&rect, &spec, &ctx);
    assert!(geo::distance_m(plan.route[0], rect.corners()[2]) < 1.0, "{}", plan.summary);
    // With the eastern half too shallow, nothing can avoid it: it says so.
    let east = CENTRE.lon;
    let depth = move |_lat: f64, lon: f64| if lon > east { 0.5 } else { 8.0 };
    let ctx = AutoContext { start: Some(boat), depth: Some(&depth), ..AutoContext::default() };
    let plan = plan_auto(&rect, &spec, &ctx);
    assert!(plan.shallow_samples > 0 && plan.summary.contains("shallow"));
}

#[test]
fn waypoints_file_for_ardupilot() {
    let mut mission = Mission::default();
    mission.add(57.0, 24.0);
    mission.add(57.001, 24.0).speed_ms = Some(0.5);
    mission.waypoints[0].hold_s = 10.0;
    let text = mission.to_waypoints_file(Some(CENTRE), 1.6);
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines[0], "QGC WPL 110");
    assert_eq!(lines.len(), 5); // home, wp, speed change, wp
    assert!(lines[2].split('\t').nth(4) == Some("10.0"));
    assert!(lines[3].split('\t').nth(3) == Some("178"));
}

#[test]
fn new_pattern_fields_round_trip() {
    let mut mission = Mission::default();
    mission.pattern = PatternSpec { pattern: Pattern::SpiralOut, swath_m: 12.0, overlap_pct: 25.0, start_corner: 3, ..PatternSpec::default() };
    let again = Mission::from_json(&mission.to_json()).unwrap();
    assert_eq!(again.pattern, mission.pattern);
}
