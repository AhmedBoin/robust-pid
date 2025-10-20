//! Cascaded PID for drone altitude control
//! Run with: cargo run --example drone_altitude

use robust_pid::{AntiWindupMode, DerivativeMode, PidBuilder, RobustPid};

struct DroneAltitudeController {
    altitude_pid: RobustPid<f64>,
    velocity_pid: RobustPid<f64>,
}

impl DroneAltitudeController {
    fn new() -> Self {
        // Outer loop: Altitude -> Velocity setpoint
        let altitude_pid = PidBuilder::new()
            .gains(1.2, 0.2, 0.5)
            .output_limits(-5.0, 5.0) // m/s limits
            .term_limits(4.0, 2.0, 3.0)
            .antiwindup(AntiWindupMode::BackCalculation, 1.0)
            .derivative_filter(DerivativeMode::LowPass, 0.25)
            .setpoint_ramping(true, 2.0)
            .time_limits(0.01, 0.05)
            .build();

        // Inner loop: Velocity -> Throttle
        let velocity_pid = PidBuilder::new()
            .gains(2.5, 0.8, 0.3)
            .output_limits(0.0, 100.0) // Throttle %
            .term_limits(80.0, 30.0, 20.0)
            .antiwindup(AntiWindupMode::BackCalculation, 1.5)
            .derivative_filter(DerivativeMode::LowPass, 0.15)
            .output_rate_limit(200.0)
            .time_limits(0.005, 0.02)
            .build();

        Self {
            altitude_pid,
            velocity_pid,
        }
    }

    fn set_target(&mut self, altitude: f64) {
        self.altitude_pid.set_setpoint(altitude);
    }

    fn update(&mut self, altitude: f64, velocity: f64, battery_v: f64, dt: f64) -> f64 {
        // Outer loop
        let altitude_out = self.altitude_pid.update(altitude, dt);
        let velocity_setpoint = altitude_out.output;

        // Inner loop
        self.velocity_pid.set_setpoint(velocity_setpoint);
        let velocity_out = self.velocity_pid.update(velocity, dt);

        // Battery compensation
        let nominal_voltage = 12.6;
        let compensation = nominal_voltage / battery_v.max(9.0);

        (velocity_out.output * compensation).clamp(0.0, 100.0)
    }
}

fn main() {
    println!("=== Drone Altitude Hold ===\n");

    let mut controller = DroneAltitudeController::new();
    controller.set_target(10.0); // 10 meters

    let mut altitude = 0.0;
    let mut velocity = 0.0;
    let mut battery = 12.6;
    let dt = 0.02; // 50 Hz

    println!("Time(s) | Alt(m) | Vel(m/s) | Throttle(%) | Battery(V)");
    println!("--------|--------|----------|-------------|------------");

    for i in 0..1000 {
        let time = i as f64 * dt;

        // Get throttle
        let throttle = controller.update(altitude, velocity, battery, dt);

        // Simulate physics
        let thrust = (throttle / 100.0) * 20.0;
        let gravity = 9.81;
        let drag = velocity * 0.1;
        let accel = thrust - gravity - drag;

        velocity += accel * dt;
        altitude += velocity * dt;
        altitude = altitude.max(0.0);

        // Wind gusts every 5 seconds
        if i % 250 == 0 && i > 0 {
            velocity += (i as f64 * 0.01).sin() * 1.5;
            println!(">> Wind gust!");
        }

        // Battery discharge
        battery -= 0.0001 * throttle * dt;

        // Change target
        if time > 10.0 && time < 10.1 {
            controller.set_target(15.0);
            println!(">> New target: 15m");
        }

        // Log every second
        if i % 50 == 0 {
            println!(
                "{:7.1} | {:6.2} | {:8.2} | {:11.1} | {:10.2}",
                time, altitude, velocity, throttle, battery
            );
        }

        // Safety check
        if battery < 9.0 {
            println!("\n⚠ Low battery - landing");
            break;
        }
    }

    println!("\n✓ Flight complete");
}
