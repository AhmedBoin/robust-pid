//! Error-based correction system using direct error input
//!
//! This example demonstrates using the PID controller with direct error signals
//! instead of setpoint tracking. Useful for systems where:
//! - Error is calculated externally (e.g., from sensor fusion, observers)
//! - You want to correct deviations from a reference state
//! - Positive error = needs more correction, Negative error = needs less
//!
//! Run with: cargo run --example error_correction

use robust_pid::{AntiWindupMode, DerivativeMode, PidBuilder};

/// Simulates a manufacturing process where alignment errors need correction
struct AlignmentCorrectionSystem {
    pid: robust_pid::RobustPid<f64>,
    actual_error: f64,
}

impl AlignmentCorrectionSystem {
    fn new() -> Self {
        // Configure PID for error correction
        let pid = PidBuilder::new()
            .gains(1.5, 0.3, 0.4)
            .output_limits(-50.0, 50.0) // Correction actuator limits
            .term_limits(40.0, 20.0, 30.0)
            .antiwindup(AntiWindupMode::BackCalculation, 1.0)
            .derivative_filter(DerivativeMode::LowPass, 0.25)
            .output_rate_limit(75.0)
            .build();

        Self {
            pid,
            actual_error: 0.0,
        }
    }

    /// Apply correction based on measured error
    /// Positive error = misaligned right, needs left correction
    /// Negative error = misaligned left, needs right correction
    fn correct(&mut self, measured_error: f64, dt: f64) -> f64 {
        // Feed the error directly to PID
        let output = self.pid.update_from_error(measured_error, dt);

        // Apply correction to the system
        self.actual_error = measured_error;

        // Simulate system response to correction
        // Positive output pushes left (reduces positive error)
        // Negative output pushes right (reduces negative error)
        let correction_effect = -output.output * 0.15 * dt;

        output.output
    }

    fn get_diagnostics(&self) -> String {
        format!(
            "Current Error: {:.2} | {}",
            self.actual_error,
            self.pid.diagnostics()
        )
    }
}

fn main() {
    println!("=== Alignment Error Correction System ===\n");
    println!("Scenario: Manufacturing robot with position sensor detecting alignment errors");
    println!("Goal: Drive alignment error to zero using error-based PID control\n");

    let mut system = AlignmentCorrectionSystem::new();

    // Initial large misalignment error (in mm)
    let mut alignment_error = 15.0; // 15mm off-center to the right
    let dt = 0.02; // 50 Hz control loop

    println!("Time(s) | Error(mm) | Correction | Status");
    println!("--------|-----------|------------|--------");

    for i in 0..600 {
        let time = i as f64 * dt;

        // Simulate external disturbances every few seconds
        if i == 100 {
            alignment_error += 8.0;
            println!(">> External disturbance: +8mm error!");
        }
        if i == 300 {
            alignment_error -= 12.0;
            println!(">> External disturbance: -12mm error!");
        }
        if i == 450 {
            alignment_error += 5.0;
            println!(">> External disturbance: +5mm error!");
        }

        // Get correction output based on current error
        let correction = system.correct(alignment_error, dt);

        // Simulate system dynamics
        // Correction output reduces the error
        alignment_error -= correction * 0.15 * dt;

        // Add small process noise
        alignment_error += (time * 3.0).sin() * 0.05;

        // Natural decay (system has some self-centering)
        alignment_error *= 0.999;

        // Log every 0.5 seconds
        if i % 25 == 0 {
            let status = if alignment_error.abs() < 0.5 {
                "✓ ALIGNED"
            } else if alignment_error.abs() < 2.0 {
                "Converging"
            } else {
                "Correcting"
            };

            println!(
                "{:7.2} | {:9.2} | {:10.2} | {}",
                time, alignment_error, correction, status
            );
        }

        // Check if we've achieved alignment
        if time > 3.0 && alignment_error.abs() < 0.1 && i % 25 == 0 {
            println!("\n✓ Precise alignment achieved at {:.2}s", time);
            println!("Final error: {:.4}mm\n", alignment_error);
        }
    }

    println!("\n{}", system.get_diagnostics());
    println!("\n=== Comparison: Error-Based vs Setpoint-Based ===");
    println!("Error-Based (this example):");
    println!("  - Feed error directly: update_from_error(error, dt)");
    println!("  - Positive error → Positive correction output");
    println!("  - Useful when error is calculated externally");
    println!("  - Example: Sensor fusion, state observers, external references");
    println!("\nSetpoint-Based (traditional):");
    println!("  - Feed measurement: update(measurement, dt)");
    println!("  - PID calculates error = setpoint - measurement");
    println!("  - Useful for direct target tracking");
    println!("  - Example: Temperature control, position tracking");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_correction_converges() {
        let mut system = AlignmentCorrectionSystem::new();
        let mut error = 10.0;
        let dt = 0.01;

        // Run for 5 seconds
        for _ in 0..500 {
            let correction = system.correct(error, dt);
            error -= correction * 0.15 * dt;
        }

        // Should converge close to zero
        assert!(error.abs() < 1.0);
    }

    #[test]
    fn test_positive_and_negative_errors() {
        let mut system = AlignmentCorrectionSystem::new();

        // Test positive error
        let correction_pos = system.correct(5.0, 0.1);
        assert!(
            correction_pos > 0.0,
            "Positive error should give positive correction"
        );

        system.pid.reset();

        // Test negative error
        let correction_neg = system.correct(-5.0, 0.1);
        assert!(
            correction_neg < 0.0,
            "Negative error should give negative correction"
        );
    }
}
