mod common;

use std::sync::Arc;

use common::{chart, Rig};
use usvgs::config::{BoatSettings, Settings};
use usvgs::core::clock;
use usvgs::core::geo::{self, LatLon};
use usvgs::core::mission::{lawnmower, Mission, Rectangle};
use usvgs::link::sim::SimLink;
use usvgs::link::BoatLink;
use usvgs::sim::boat::Catamaran;
use usvgs::sim::oil::{appearance, OilSpill, Skimmer};
use usvgs::sim::world::World;
use usvgs::vehicle::autopilot::mix;
use usvgs::vehicle::vehicle::Operator;
use usvgs::vehicle::{Mode, Vehicle};

const OPEN_WATER: LatLon = LatLon::new(57.10, 23.95);

fn world(wind: f64, current: f64) -> World {
    let mut w = World::new(Some(chart()), OPEN_WATER, 7);
    w.wind_speed_ms = wind;
    w.current_speed_ms = current;
    w
}

fn drive(port: f64, stbd: f64, deflection: f64) -> Operator {
    let (throttle, steer) = ((port + stbd) / 2.0, (port - stbd) / 2.0);
    Operator { port, stbd, throttle, steer, deflection, connected: true }
}

// -- the boat ---------------------------------------------------------------------

#[test]
fn full_ahead_reaches_top_speed_on_heading() {
    let w = world(0.0, 0.0);
    let mut b = Catamaran::new(BoatSettings::default(), &w, OPEN_WATER, 0.0);
    for _ in 0..1000 {
        b.step(&w, 0.02, 1.0, 1.0);
    }
    assert!((b.state.u - 2.37).abs() < 0.1, "u = {}", b.state.u);
    assert!(geo::angle_difference(b.heading_deg(), 0.0).abs() < 0.5);
    assert!(b.state.battery_current_a > 30.0);
}

#[test]
fn differential_thrust_turns_to_starboard() {
    let w = world(0.0, 0.0);
    let mut b = Catamaran::new(BoatSettings::default(), &w, OPEN_WATER, 0.0);
    for _ in 0..100 {
        b.step(&w, 0.02, 1.0, -1.0);
    }
    assert!(b.state.r > 0.3);
    assert!(0.0 < b.heading_deg() && b.heading_deg() < 180.0);
}

#[test]
fn current_carries_a_stopped_boat() {
    let mut w = world(0.0, 0.5);
    w.current_to_deg = 90.0;
    let mut b = Catamaran::new(BoatSettings::default(), &w, OPEN_WATER, 0.0);
    for _ in 0..500 {
        b.step(&w, 0.02, 0.0, 0.0);
    }
    assert!((b.state.east - 5.0).abs() < 0.3, "east = {}", b.state.east);
}

#[test]
fn boat_cannot_drive_onto_land() {
    let w = world(0.0, 0.0);
    // Daugavgrīva, a few hundred metres off the beach, heading east for the shore.
    let mut b = Catamaran::new(BoatSettings::default(), &w, LatLon::new(57.048, 23.995), 90.0);
    for _ in 0..30000 {
        b.step(&w, 0.02, 1.0, 1.0);
        if b.state.grounded {
            break;
        }
    }
    assert!(b.state.grounded);
    assert!(!b.depth_here(&w).is_nan(), "it stopped in the water, not on land");
}

#[test]
fn link_failsafe_stops_motors() {
    let link = SimLink::new(Settings::default().shared(), Some(chart()), 7);
    link.send_command(1.0, 1.0, true);
    for _ in 0..25 {
        link.step(0.02);
    }
    assert!(link.core().boat.state.motor_port > 0.5);
    for _ in 0..150 {
        link.step(0.02);
    }
    assert!(link.telemetry().onboard_failsafe);
    assert!(link.core().boat.state.motor_port.abs() < 0.05);
}

#[test]
fn telemetry_reports_depth_and_water() {
    let link = SimLink::new(Settings::default().shared(), Some(chart()), 7);
    let t = link.telemetry();
    assert!(t.depth_m.is_some_and(|d| d > 0.5));
    assert!(t.water.salinity_psu.is_some_and(|s| 0.1 < s && s < 8.0));
    assert!(t.has_position());
}

// -- the vehicle --------------------------------------------------------------------

#[test]
fn disarmed_boat_does_not_move() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 1.0));
    let nav = rig.run(3.0, 0.05);
    assert!(!nav.armed);
    assert!(nav.telemetry.as_ref().unwrap().throttle_port.abs() < 0.01);
}

#[test]
fn manual_drive() {
    let rig = Rig::new(3.0, 0.1);
    assert!(rig.vehicle.lock().arm());
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 1.0));
    let nav = rig.run(10.0, 0.05);
    assert!(nav.telemetry.as_ref().unwrap().sog_ms.unwrap() > 1.8);
}

#[test]
fn survey_mission_runs_and_tracks_the_lanes() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().arm();
    let start = rig.position();
    let rect = Rectangle::new(geo::destination(start, 300.0, 90.0), 120.0, 40.0, 30.0);
    {
        let mut v = rig.vehicle.lock();
        v.mission.area = Some(rect);
        v.mission.set_route(&lawnmower(&rect, 10.0, 2.0, true), None);
        assert!(v.set_mode(Mode::Auto));
    }
    let mut worst: f64 = 0.0;
    let mut nav = rig.vehicle.nav();
    for _ in 0..900 {
        nav = rig.run(1.0, 0.05);
        if let (Some(xte), Some(index)) = (nav.cross_track_m, nav.waypoint_index) {
            if index > 1 {
                worst = worst.max(xte.abs());
            }
        }
        if nav.mode != Mode::Auto {
            break;
        }
    }
    assert_eq!(nav.mode, Mode::Hold);
    assert!(worst < 8.0, "worst cross-track {worst}");
    assert!(nav.coverage > 0.7, "coverage {}", nav.coverage);
}

#[test]
fn stick_override_takes_manual_control() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_mode(Mode::Hold);
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 0.9));
    let nav = rig.run(0.2, 0.05);
    assert_eq!(nav.mode, Mode::Manual);
}

#[test]
fn point_nav_arrives_and_holds() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().arm();
    let target = geo::destination(rig.position(), 330.0, 80.0);
    rig.vehicle.lock().go_to(target.lat, target.lon);
    let mut nav = rig.vehicle.nav();
    for _ in 0..200 {
        nav = rig.run(1.0, 0.05);
        if nav.target_label.contains("holding") {
            break;
        }
    }
    assert!(nav.target_label.contains("holding"), "label {:?}", nav.target_label);
    rig.run(30.0, 0.05);
    assert!(geo::distance_m(rig.position(), target) < 8.0);
}

#[test]
fn geofence_returns_home() {
    let mut s = Settings::default();
    s.sim.gps_noise_m = 0.2;
    s.sim.wind_speed_ms = 3.0;
    s.sim.current_speed_ms = 0.1;
    s.safety.geofence_radius_m = 40.0;
    let rig = Rig::with(s);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 0.3));
    let mut nav = rig.vehicle.nav();
    for _ in 0..60 {
        nav = rig.run(1.0, 0.05);
        if nav.mode == Mode::ReturnHome {
            break;
        }
    }
    assert_eq!(nav.mode, Mode::ReturnHome);
    assert!(nav.has_alarm("geofence"));
}

#[test]
fn battery_critical_returns_home() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_mode(Mode::Hold);
    rig.link.core().boat.state.battery_soc = 0.10;
    let nav = rig.run(1.0, 0.05);
    assert_eq!(nav.mode, Mode::ReturnHome);
}

#[test]
fn shallow_water_stops_autonomous_modes() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().arm();
    // Point Nav straight at the beach (24.006 °E is still a metre deep;
    // 24.012 °E is ashore).
    rig.vehicle.lock().go_to(57.048, 24.012);
    let mut nav = rig.vehicle.nav();
    for _ in 0..1200 {
        nav = rig.run(1.0, 0.05);
        if nav.mode != Mode::PointNav {
            break;
        }
    }
    assert_eq!(nav.mode, Mode::Stop);
    assert_eq!(rig.vehicle.lock().groundings, 0);
}

#[test]
fn link_loss_then_hold() {
    let rig = Rig::new(3.0, 0.1);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_mode(Mode::Hold);
    rig.vehicle.lock().set_mode(Mode::Manual);
    // Stop delivering telemetry and fake five silent seconds.
    clock::set_offset(5.0);
    let nav = rig.vehicle.tick(0.05);
    assert!(!nav.link_ok);
    assert!(nav.has_alarm("link"));
    clock::set_offset(0.0);
    let nav = rig.run(0.5, 0.05);
    assert!(nav.link_ok);
    assert_eq!(nav.mode, Mode::Hold);
}

#[test]
fn vehicle_estimates_wind_and_current() {
    let mut s = Settings::default();
    s.sim.wind_speed_ms = 5.0;
    s.sim.wind_from_deg = 270.0;
    s.sim.current_speed_ms = 0.3;
    s.sim.current_to_deg = 180.0;
    let settings = s.shared();
    let link = Arc::new(SimLink::new(settings.clone(), Some(chart()), 7));
    let vehicle = Vehicle::new(settings, link.clone() as Arc<dyn BoatLink>, None, false);
    for i in 0..1200 {
        link.step(0.05);
        if i % 2 == 0 {
            link.deliver(link.telemetry());
        }
        if i == 10 {
            vehicle.lock().arm();
            vehicle.lock().set_operator(drive(0.6, 0.6, 0.3));
        }
        vehicle.tick(0.05);
    }
    let nav = vehicle.nav();
    let (wind, from) = nav.true_wind.unwrap();
    assert!((wind - 5.0).abs() < 1.5, "wind {wind}");
    assert!(geo::angle_difference(from, 270.0).abs() < 25.0, "from {from}");
    assert!((nav.current.unwrap().0 - 0.3).abs() < 0.15, "current {:?}", nav.current);
}

#[test]
fn mixer_keeps_steering() {
    let (port, stbd) = mix(1.0, 0.5);
    assert!((port - stbd - 1.0).abs() < 1e-9);
}

// -- oil ------------------------------------------------------------------------------

fn oil_world(wind: f64, current: f64) -> World {
    let mut w = World::new(Some(chart()), OPEN_WATER, 7);
    w.wind_speed_ms = wind;
    w.wind_from_deg = 270.0;
    w.current_speed_ms = current;
    w.current_to_deg = 0.0;
    w
}

fn mean(v: &[f64]) -> f64 {
    v.iter().sum::<f64>() / v.len() as f64
}

#[test]
fn slick_drifts_with_current_and_three_percent_of_wind() {
    let mut w = oil_world(10.0, 0.1);
    let mut spill = OilSpill::new(&w, 11);
    spill.release(&w, OPEN_WATER, 100.0, 20.0, 400);
    let (n0, e0) = (mean(&spill.north), mean(&spill.east));
    for _ in 0..600 {
        w.gust = 0.0;
        spill.step(&w, 0.1);
    }
    // 60 s: current 0.1 m/s north, windage 0.3 m/s east (wind from the west).
    assert!((mean(&spill.north) - n0 - 6.0).abs() < 1.0);
    assert!((mean(&spill.east) - e0 - 18.0).abs() < 1.5);
    assert!(spill.totals.evaporated_l > 0.0);
}

#[test]
fn skimmer_recovers_oil_and_fills_the_tank() {
    let settings = Settings::default().skimmer;
    let w = oil_world(0.0, 0.0);
    let mut spill = OilSpill::new(&w, 11);
    spill.release(&w, OPEN_WATER, 200.0, 10.0, 800);
    let mut skimmer = Skimmer::new(settings.clone());
    skimmer.deployed = true;
    for _ in 0..600 {
        skimmer.step(0.1, &mut spill, 0.0, 0.0, 0.0, 0.3, false);
    }
    assert!(skimmer.recovered_oil_l > 1.0);
    assert!((spill.totals.recovered_l - skimmer.recovered_oil_l).abs() < 1e-9);
    assert!(skimmer.rate_lpm <= settings.max_rate_lpm * settings.oil_content + 1e-6);
    assert!((skimmer.tank_l - skimmer.recovered_oil_l / settings.oil_content).abs() < 1e-6);
}

#[test]
fn skimming_too_fast_loses_oil() {
    let skimmer = Skimmer::new(Settings::default().skimmer);
    assert!(skimmer.efficiency_at(0.3) > 0.8);
    assert!(skimmer.efficiency_at(1.5) < 0.2);
}

#[test]
fn tank_offloads_alongside_with_skimmer_stowed() {
    let mut skimmer = Skimmer::new(Settings::default().skimmer);
    skimmer.tank_l = 30.0;
    let w = oil_world(0.0, 0.0);
    let mut spill = OilSpill::new(&w, 11);
    skimmer.step(10.0, &mut spill, 0.0, 0.0, 0.0, 0.0, true);
    assert!(skimmer.offloading && skimmer.tank_l < 30.0);
}

#[test]
fn bonn_codes() {
    assert_eq!(appearance(Some(0.1)).0, "1");
    assert_eq!(appearance(Some(2.0)).0, "2");
    assert_eq!(appearance(Some(100.0)).0, "4");
    assert_eq!(appearance(Some(0.0)).0, "—");
}

#[test]
fn sweep_route_moves_with_the_water() {
    let mut mission = Mission::default();
    mission.area = Some(Rectangle::new(OPEN_WATER, 40.0, 20.0, 90.0));
    mission.regenerate();
    let first = mission.waypoints[0].position();
    mission.shift(10.0, 0.0);
    let moved = mission.waypoints[0].position();
    assert!((geo::distance_m(first, moved) - 10.0).abs() < 0.01);
    assert!(geo::bearing_deg(first, moved).abs() < 0.5);
}

#[test]
fn simulated_skimmer_draws_power() {
    let settings = Settings::default();
    let power = settings.skimmer.power_a;
    let link = SimLink::new(settings.shared(), Some(chart()), 7);
    link.step(0.1);
    let idle = link.core().boat.state.battery_current_a;
    link.send_payload("skimmer", true);
    for _ in 0..5 {
        link.step(0.1);
    }
    assert!((link.core().boat.state.battery_current_a - (idle + power)).abs() < 0.2);
    assert!(link.telemetry().skimmer.deployed);
}

#[test]
fn shore_power_charges_alongside_and_banks_report() {
    let link = SimLink::new(Settings::default().shared(), Some(chart()), 7);
    link.core().boat.state.battery_soc = 0.5;
    for _ in 0..200 {
        link.step(0.05);
    }
    let soc = link.core().boat.state.battery_soc;
    assert!(soc > 0.5, "charging alongside: {soc}");
    let power = link.telemetry().power.unwrap();
    assert!(power.shore_connected && power.shore_w > 50.0);
    assert_eq!(power.banks.len(), 2);
    let mean = (power.banks[0].soc_pct + power.banks[1].soc_pct) / 2.0;
    assert!((mean - soc * 100.0).abs() < 0.5);
}

#[test]
fn hard_running_warms_the_banks_and_draws_thruster_power() {
    let w = World::new(Some(chart()), OPEN_WATER, 7);
    let mut b = Catamaran::new(BoatSettings::default(), &w, OPEN_WATER, 0.0);
    for _ in 0..30_000 {
        b.step(&w, 0.02, 1.0, 1.0);
    }
    let p = b.power_state(false);
    assert!(p.port_w > 200.0 && p.stbd_w > 200.0);
    assert!(b.banks.temp_c[0] > 14.0);
    let banks = b.bank_states();
    assert!(banks[0].amps > 0.0 && banks[0].cell_max_v > banks[0].cell_min_v);
}

// -- motion limits ----------------------------------------------------------------

fn limited_rig(max_speed: f64, accel: f64, turn: f64) -> Rig {
    let mut s = Settings::default();
    s.sim.gps_noise_m = 0.2;
    s.sim.wind_speed_ms = 0.0;
    s.sim.current_speed_ms = 0.0;
    s.limits.max_speed_ms = max_speed;
    s.limits.max_accel_ms2 = accel;
    s.limits.max_turn_dps = turn;
    Rig::with(s)
}

#[test]
fn the_sticks_cannot_drive_past_the_speed_limit() {
    let rig = limited_rig(1.0, 2.0, 60.0);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 1.0));
    let mut top: f64 = 0.0;
    for _ in 0..30 {
        let nav = rig.run(1.0, 0.05);
        top = top.max(nav.telemetry.as_ref().unwrap().sog_ms.unwrap());
    }
    assert!(top < 1.2, "top speed {top}");
    assert!(top > 0.8, "but it does get going: {top}");
}

#[test]
fn acceleration_is_limited() {
    // 0.2 m/s²: after two seconds the throttle is at most a sixth.
    let rig = limited_rig(2.3, 0.2, 60.0);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 1.0));
    let nav = rig.run(2.0, 0.05);
    assert!(nav.command_port < 0.2, "command {}", nav.command_port);
    assert_eq!(nav.limit, Some(usvgs::vehicle::limits::Limit::Accel));
    // Unlimited it would be at full throttle by now.
    let free = limited_rig(2.3, 100.0, 60.0);
    free.vehicle.lock().arm();
    free.vehicle.lock().set_operator(drive(1.0, 1.0, 1.0));
    assert!(free.run(2.0, 0.05).command_port > 0.99);
}

#[test]
fn rate_of_turn_is_limited() {
    let rig = limited_rig(2.3, 5.0, 15.0);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_operator(drive(1.0, -1.0, 1.0)); // a hard pivot
    let mut fastest: f64 = 0.0;
    for _ in 0..10 {
        let nav = rig.run(1.0, 0.05);
        fastest = fastest.max(nav.telemetry.as_ref().unwrap().rot_dps.unwrap().abs());
    }
    assert!(fastest < 22.0, "turned at {fastest} °/s against a 15 °/s limit");
    assert!(fastest > 8.0);
}

#[test]
fn usv_stop_is_never_limited() {
    let rig = limited_rig(2.3, 0.05, 60.0);
    rig.vehicle.lock().arm();
    rig.vehicle.lock().set_operator(drive(1.0, 1.0, 1.0));
    rig.run(20.0, 0.05);
    rig.vehicle.lock().set_mode(Mode::Stop);
    let nav = rig.run(0.1, 0.05);
    assert_eq!((nav.command_port, nav.command_stbd), (0.0, 0.0));
}
