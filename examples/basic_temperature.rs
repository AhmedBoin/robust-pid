//! Simple temperature control example
//! Run with: cargo run --example basic_temperature

use robust_pid::{AntiWindupMode, DerivativeMode, PidBuilder};

fn main() {
    println!("=== Basic Temperature Control ===\n");

    // Create a PID controller for temperature control
    let mut pid = PidBuilder::new()
        .gains(2.0, 0.5, 0.1)
        .output_limits(-100.0, 100.0)
        .term_limits(80.0, 50.0, 20.0)
        .antiwindup(AntiWindupMode::BackCalculation, 1.0)
        .derivative_filter(DerivativeMode::LowPass, 0.2)
        .build();

    // Target temperature: 25°C
    pid.set_setpoint(25.0);

    // Simulate heating system
    let mut temperature = 20.0;
    let dt = 0.1; // 100ms update rate

    println!("Time(s) | Temp(°C) | Output(%) | Error(°C) | Status");
    println!("--------|----------|-----------|-----------|--------");

    for i in 0..200 {
        let time = i as f64 * dt;

        // Add some noise to simulation
        let noise = if i % 30 == 0 {
            (i as f64 * 0.1).sin() * 0.5
        } else {
            0.0
        };

        // Get control output
        let output = pid.update(temperature, dt);

        // Simulate heater response
        temperature += output.output * 0.01 + noise;
        temperature -= (temperature - 20.0) * 0.02; // Heat loss

        // Print every second
        if i % 10 == 0 {
            println!(
                "{:7.1} | {:8.2} | {:9.2} | {:9.2} | {:?}",
                time, temperature, output.output, output.error, output.safety_status
            );
        }

        // Check if we've reached steady state
        if time > 10.0 && output.error.abs() < 0.1 {
            println!("\n✓ Steady state reached at {:.1}s", time);
            break;
        }
    }

    // Print diagnostics
    println!("\n{}", pid.diagnostics());
}
