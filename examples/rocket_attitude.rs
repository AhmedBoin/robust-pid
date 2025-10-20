//! Three-axis rocket attitude control
//! Run with: cargo run --example rocket_attitude

use robust_pid::{AntiWindupMode, DerivativeMode, PidBuilder, RobustPid, SafetyStatus};

struct RocketController {
    pitch_pid: RobustPid<f64>,
    yaw_pid: RobustPid<f64>,
    roll_pid: RobustPid<f64>,
    emergency_shutdown: bool,
}

impl RocketController {
    fn new() -> Self {
        // Aggressive tuning for fast rocket response
        let pitch_yaw_config = PidBuilder::new()
            .gains(5.0, 1.5, 2.0)
            .output_limits(-15.0, 15.0)
            .term_limits(12.0, 5.0, 8.0)
            .antiwindup(AntiWindupMode::BackCalculation, 2.0)
            .derivative_filter(DerivativeMode::LowPass, 0.15)
            .output_rate_limit(100.0)
            .time_limits(0.001, 0.01)
            .build();

        let roll_config = PidBuilder::new()
            .gains(3.0, 0.8, 1.0)
            .output_limits(-30.0, 30.0)
            .term_limits(20.0, 10.0, 15.0)
            .antiwindup(AntiWindupMode::BackCalculation, 1.5)
            .derivative_filter(DerivativeMode::LowPass, 0.2)
            .output_rate_limit(150.0)
            .time_limits(0.001, 0.01)
            .build();

        Self {
            pitch_pid: pitch_yaw_config.clone(),
            yaw_pid: pitch_yaw_config,
            roll_pid: roll_config,
            emergency_shutdown: false,
        }
    }

    fn set_attitude(&mut self, pitch: f64, yaw: f64, roll: f64) {
        self.pitch_pid.set_setpoint(pitch);
        self.yaw_pid.set_setpoint(yaw);
        self.roll_pid.set_setpoint(roll);
    }

    fn update(&mut self, pitch: f64, yaw: f64, roll: f64, dt: f64) -> (f64, f64, f64) {
        if self.emergency_shutdown {
            return (0.0, 0.0, 0.0);
        }

        let pitch_out = self.pitch_pid.update(pitch, dt);
        let yaw_out = self.yaw_pid.update(yaw, dt);
        let roll_out = self.roll_pid.update(roll, dt);

        // Safety check
        if matches!(pitch_out.safety_status, SafetyStatus::MultipleFaults)
            || matches!(yaw_out.safety_status, SafetyStatus::MultipleFaults)
        {
            self.emergency_shutdown = true;
            eprintln!("EMERGENCY SHUTDOWN: Control system failure");
        }

        if pitch_out.error.abs() > 45.0 || yaw_out.error.abs() > 45.0 {
            self.emergency_shutdown = true;
            eprintln!("EMERGENCY SHUTDOWN: Attitude error too large");
        }

        (pitch_out.output, yaw_out.output, roll_out.output)
    }

    fn is_safe(&self) -> bool {
        !self.emergency_shutdown
    }
}

fn main() {
    println!("=== Rocket Launch Simulation ===\n");

    let mut controller = RocketController::new();
    controller.set_attitude(90.0, 0.0, 0.0); // Vertical

    let mut pitch = 89.5;
    let mut yaw = 0.2;
    let mut roll = 0.0;
    let dt = 0.005; // 200 Hz

    println!("Time(s) | Pitch(°) | Yaw(°) | Roll(°) | TVC-P(°) | TVC-Y(°)");
    println!("--------|----------|--------|---------|----------|----------");

    for i in 0..4000 {
        let time = i as f64 * dt;

        // Gravity turn maneuver after 10 seconds
        if time > 10.0 {
            let target_pitch = 90.0 - (time - 10.0) * 2.0;
            controller.set_attitude(target_pitch.max(45.0), 0.0, 0.0);
        }

        // Get thrust vector commands
        let (tvc_pitch, tvc_yaw, tvc_roll) = controller.update(pitch, yaw, roll, dt);

        if !controller.is_safe() {
            println!("\n⚠ LAUNCH ABORTED at T+{:.1}s", time);
            break;
        }

        // Simulate rocket dynamics
        pitch += tvc_pitch * 0.1 * dt;
        yaw += tvc_yaw * 0.1 * dt;
        roll += tvc_roll * 0.05 * dt;

        // Wind and aerodynamic disturbances
        pitch += (time * 2.0).sin() * 0.01;
        yaw += (time * 1.5).cos() * 0.008;

        // Log every 1 second
        if i % 200 == 0 {
            println!(
                "{:7.1} | {:8.2} | {:6.2} | {:7.2} | {:8.2} | {:8.2}",
                time, pitch, yaw, roll, tvc_pitch, tvc_yaw
            );
        }
    }

    println!("\n🚀 Launch sequence complete");
}
