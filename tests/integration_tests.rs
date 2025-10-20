use num_traits::Float;
use robust_pid::{AntiWindupMode, DerivativeMode, PidBuilder, SafetyStatus};

#[test]
fn test_basic_control_loop() {
    let mut pid = PidBuilder::new()
        .gains(1.0, 0.1, 0.05)
        .output_limits(-10.0, 10.0)
        .build();

    pid.set_setpoint(5.0);

    let mut value = 0.0;
    for _ in 0..100 {
        let output = pid.update(value, 0.1);
        value += output.output * 0.1;
    }

    // Should converge close to setpoint
    assert!((value - 5.0).abs() < 0.5);
}

#[test]
fn test_anti_windup() {
    let mut pid = PidBuilder::new()
        .gains(1.0, 1.0, 0.0)
        .output_limits(-10.0, 10.0)
        .term_limits(100.0, 5.0, 100.0) // Limited integral
        .antiwindup(AntiWindupMode::Clamping, 1.0)
        .build();

    pid.set_setpoint(100.0);

    // Force saturation
    for _ in 0..20 {
        pid.update(0.0, 1.0);
    }

    let diag = pid.diagnostics();
    assert!(diag.current_integral <= 5.0);
}

#[test]
fn test_setpoint_ramping() {
    let mut pid = PidBuilder::new()
        .gains(1.0, 0.0, 0.0)
        .output_limits(-100.0, 100.0)
        .setpoint_ramping(true, 10.0) // 10 units/sec
        .build();

    pid.set_setpoint(100.0);

    // After 1 second, should only be at 10
    let output = pid.update(0.0, 1.0);
    assert!((output.effective_setpoint - 10.0).abs() < 0.1);
}

#[test]
fn test_derivative_filter() {
    let mut pid = PidBuilder::new()
        .gains(0.0, 0.0, 1.0)
        .output_limits(-100.0, 100.0)
        .derivative_filter(DerivativeMode::LowPass, 0.5)
        .build();

    pid.set_setpoint(0.0);

    // Create a step change in measurement to generate derivative
    let out1 = pid.update(0.0, 0.1); // First measurement: 0
    let out2 = pid.update(10.0, 0.1); // Step to 10 - should have large negative derivative
    let out3 = pid.update(10.0, 0.1); // Hold at 10 - derivative should decay due to filtering

    // First output has no derivative (no previous measurement)
    assert_eq!(out1.d, 0.0);

    // Second output should have significant derivative due to step change
    // derivative = -(10.0 - 0.0) / 0.1 = -100, then scaled by kd=1.0
    assert!(out2.d.abs() > 0.0, "Step change should produce derivative");

    // Third output should have smaller derivative due to low-pass filtering
    // Even though measurement is constant, the filter causes decay
    assert!(
        out3.d.abs() < out2.d.abs(),
        "Filtered derivative should decay: out2.d={}, out3.d={}",
        out2.d,
        out3.d
    );
}

// Alternative test that's more comprehensive
#[test]
fn test_derivative_filter_comprehensive() {
    // Test with filter
    let mut pid_filtered = PidBuilder::new()
        .gains(0.0, 0.0, 1.0)
        .output_limits(-100.0, 100.0)
        .derivative_filter(DerivativeMode::LowPass, 0.3)
        .build();

    // Test without filter
    let mut pid_unfiltered = PidBuilder::new()
        .gains(0.0, 0.0, 1.0)
        .output_limits(-100.0, 100.0)
        .derivative_filter(DerivativeMode::None, 0.0)
        .build();

    pid_filtered.set_setpoint(0.0);
    pid_unfiltered.set_setpoint(0.0);

    // Apply same measurement sequence to both
    let measurements = vec![0.0, 10.0, 12.0, 11.0, 10.5];
    let mut filtered_derivatives = Vec::new();
    let mut unfiltered_derivatives = Vec::new();

    for &measurement in &measurements {
        let out_f = pid_filtered.update(measurement, 0.1);
        let out_u = pid_unfiltered.update(measurement, 0.1);
        filtered_derivatives.push(out_f.d);
        unfiltered_derivatives.push(out_u.d);
    }

    // After the first measurement, filtered derivatives should generally be
    // smoother (smaller magnitude changes) than unfiltered ones
    // Check that the filtered version has less variation
    let mut filtered_variation = 0.0;
    let mut unfiltered_variation = 0.0;

    for i in 2..filtered_derivatives.len() {
        filtered_variation += (filtered_derivatives[i] - filtered_derivatives[i - 1]).abs();
        unfiltered_variation += (unfiltered_derivatives[i] - unfiltered_derivatives[i - 1]).abs();
    }

    assert!(
        filtered_variation < unfiltered_variation,
        "Filtered derivative should have less variation. Filtered: {}, Unfiltered: {}",
        filtered_variation,
        unfiltered_variation
    );
}

// Test specifically for noise rejection
#[test]
fn test_derivative_noise_rejection() {
    let mut pid = PidBuilder::new()
        .gains(0.0, 0.0, 1.0)
        .output_limits(-100.0, 100.0)
        .derivative_filter(DerivativeMode::LowPass, 0.2) // Strong filtering
        .build();

    pid.set_setpoint(10.0);

    // Simulate noisy measurement with underlying ramp
    let mut outputs = Vec::new();
    for i in 0..10 {
        let base_value = i as f64; // Underlying ramp
        let noise = if i % 2 == 0 { 0.5 } else { -0.5 }; // Alternating noise
        let measurement = base_value + noise;

        let output = pid.update(measurement, 0.1);
        outputs.push(output.d);
    }

    // The filtered derivative should be relatively smooth despite noise
    // Check that no single derivative spike is too large
    let max_derivative = outputs.iter().map(|d| d.abs()).fold(0.0, f64::max);

    // Without filtering, alternating noise would cause large derivative swings
    // With filtering (alpha=0.2), the derivative should be bounded
    assert!(
        max_derivative < 50.0,
        "Filtered derivative should reject noise: max = {}",
        max_derivative
    );
}

#[test]
fn test_output_rate_limiting() {
    let mut pid = PidBuilder::new()
        .gains(10.0, 0.0, 0.0)
        .output_limits(-100.0, 100.0)
        .output_rate_limit(50.0) // 50 units/sec max
        .build();

    pid.set_setpoint(100.0);

    let out = pid.update(0.0, 0.1);
    // Max change in 0.1s = 50 * 0.1 = 5
    assert!(out.output <= 5.0);
}

#[test]
fn test_safety_status() {
    let mut pid = PidBuilder::new()
        .gains(100.0, 0.0, 0.0)
        .output_limits(-10.0, 10.0)
        .build();

    pid.set_setpoint(100.0);

    let output = pid.update(0.0, 1.0);
    assert!(matches!(
        output.safety_status,
        SafetyStatus::OutputSaturated | SafetyStatus::MultipleFaults
    ));
}

#[test]
fn test_reset() {
    let mut pid = PidBuilder::new()
        .gains(1.0, 1.0, 1.0)
        .output_limits(-100.0, 100.0)
        .build();

    pid.set_setpoint(10.0);

    // Build up state
    for _ in 0..10 {
        pid.update(0.0, 1.0);
    }

    // Reset
    pid.reset();
    let diag = pid.diagnostics();

    assert_eq!(diag.current_integral, 0.0);
    assert_eq!(diag.current_derivative, 0.0);
}
