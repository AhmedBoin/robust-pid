//! Aircraft pitch control simulation
//! Run with: cargo run --example aircraft_pitch

use robust_pid::{AntiWindupMode, DerivativeMode, PidBuilder, SafetyStatus};

fn main() {
    println!("=== Aircraft Pitch Control ===\n");

    let mut pid = create_pitch_controller();

    // Flight scenario: Take off and climb to cruise
    pid.set_setpoint(0.0); // Start level

    let mut pitch = 0.0;
    let mut pitch_rate = 0.0;
    let dt = 0.01; // 100 Hz control loop

    println!("Time(s) | Pitch(°) | Target(°) | Elevator(°) | Status");
    println!("--------|----------|-----------|-------------|--------");

    for i in 0..1000 {
        let time = i as f64 * dt;

        // Flight profile
        if time > 2.0 && time < 2.1 {
            pid.set_setpoint(10.0); // Climb attitude
            println!(">> Initiating climb to 10°");
        }
        if time > 6.0 && time < 6.1 {
            pid.set_setpoint(5.0); // Level off to cruise
            println!(">> Leveling off to 5°");
        }

        // Add atmospheric turbulence
        let turbulence = if i % 50 == 0 {
            (time * 3.0).sin() * 0.3
        } else {
            0.0
        };

        // Get control output
        let output = pid.update(pitch, dt);

        // Simulate aircraft pitch dynamics
        let elevator = output.output;
        let pitch_acceleration = elevator * 0.8 - pitch_rate * 0.1 + turbulence;
        pitch_rate += pitch_acceleration * dt;
        pitch += pitch_rate * dt;

        // Log every 0.5 seconds
        if i % 50 == 0 {
            println!(
                "{:7.2} | {:8.2} | {:9.2} | {:11.2} | {:?}",
                time, pitch, output.effective_setpoint, elevator, output.safety_status
            );

            // Check for critical issues
            if matches!(output.safety_status, SafetyStatus::MultipleFaults) {
                println!("\n⚠ CRITICAL: Multiple control faults detected!");
                break;
            }
        }
    }

    println!("\n{}", pid.diagnostics());
}

fn create_pitch_controller() -> robust_pid::RobustPid<f64> {
    PidBuilder::new()
        .gains(3.5, 0.8, 1.2)
        .output_limits(-30.0, 30.0)
        .term_limits(25.0, 10.0, 15.0)
        .antiwindup(AntiWindupMode::BackCalculation, 1.5)
        .derivative_filter(DerivativeMode::LowPass, 0.2)
        .output_rate_limit(50.0)
        .setpoint_ramping(true, 20.0)
        .time_limits(0.001, 0.1)
        .build()
}
