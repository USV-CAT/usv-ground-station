//! Control modes, named the way USV ground stations name them.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Sticks straight to the motors.
    Manual,
    /// Speed held by the autopilot; the operator steers.
    SpeedHold,
    /// Speed and course both held; the operator nudges them.
    CourseHold,
    /// Go to one point on the chart, then keep station there.
    PointNav,
    /// Run the planned mission.
    Auto,
    /// Keep station where the boat is.
    Hold,
    /// Return to the launch point, then keep station there.
    ReturnHome,
    /// All stop: motors at zero, the boat drifts.
    Stop,
}

impl Mode {
    pub const ALL: [Mode; 8] = [
        Mode::Manual,
        Mode::SpeedHold,
        Mode::CourseHold,
        Mode::PointNav,
        Mode::Auto,
        Mode::Hold,
        Mode::ReturnHome,
        Mode::Stop,
    ];

    pub fn value(self) -> &'static str {
        match self {
            Mode::Manual => "manual",
            Mode::SpeedHold => "speed_hold",
            Mode::CourseHold => "course_hold",
            Mode::PointNav => "point_nav",
            Mode::Auto => "auto",
            Mode::Hold => "hold",
            Mode::ReturnHome => "return_home",
            Mode::Stop => "stop",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Mode::Manual => "Manual",
            Mode::SpeedHold => "Speed Hold",
            Mode::CourseHold => "Speed/Course Hold",
            Mode::PointNav => "Point Nav",
            Mode::Auto => "Preprogram",
            Mode::Hold => "Station Hold",
            Mode::ReturnHome => "Return Home",
            Mode::Stop => "USV Stop",
        }
    }

    pub fn autonomous(self) -> bool {
        matches!(self, Mode::PointNav | Mode::Auto | Mode::Hold | Mode::ReturnHome)
    }

    pub fn operator_steers(self) -> bool {
        matches!(self, Mode::Manual | Mode::SpeedHold)
    }
}
