//! # Robust PID Controller for Critical Vehicle Systems
//!
//! A production-grade PID controller designed for aerospace, automotive, and other
//! safety-critical applications. Features comprehensive safety mechanisms, diagnostics,
//! and real-time adaptability.
//!
//! ## Key Features
//! - **Anti-windup protection** with multiple strategies
//! - **Derivative filtering** to reduce noise sensitivity
//! - **Setpoint ramping** to prevent aggressive changes
//! - **Safety monitoring** with error detection
//! - **Variable time step** support for real-time systems
//! - **Bumpless transfer** when changing modes
//! - **Output rate limiting** for actuator protection
//! - **Comprehensive diagnostics** for system analysis

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use core::fmt;
use num_traits::Float;

/// Safety status returned by the controller
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub enum SafetyStatus {
    /// Normal operation
    Normal,
    /// Output is saturated (at limit)
    OutputSaturated,
    /// Integral term is saturated
    IntegralWindup,
    /// Derivative term is noisy/unstable
    DerivativeNoisy,
    /// Time step is too large
    TimeStepExcessive,
    /// Multiple safety issues detected
    MultipleFaults,
}

/// Anti-windup strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub enum AntiWindupMode {
    /// Clamp integral when output saturates
    Clamping,
    /// Back-calculate integral when saturated (recommended)
    BackCalculation,
    /// Conditional integration (don't integrate when saturated)
    Conditional,
}

/// Derivative filtering mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub enum DerivativeMode {
    /// No filtering (not recommended for noisy systems)
    None,
    /// First-order low-pass filter (recommended)
    LowPass,
    /// Simple moving average (fixed alpha)
    SimpleMovingAverage,
}

/// Configuration for the PID controller
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PidConfig<T: Float> {
    /// Proportional gain
    pub kp: T,
    /// Integral gain
    pub ki: T,
    /// Derivative gain
    pub kd: T,

    /// Maximum allowed output
    pub output_max: T,
    /// Minimum allowed output
    pub output_min: T,

    /// Maximum rate of change of output (per second)
    pub output_rate_limit: Option<T>,

    /// Proportional term limit
    pub p_limit: T,
    /// Integral term limit (anti-windup)
    pub i_limit: T,
    /// Derivative term limit
    pub d_limit: T,

    /// Anti-windup strategy
    pub antiwindup_mode: AntiWindupMode,
    /// Back-calculation gain (for BackCalculation mode)
    pub antiwindup_gain: T,

    /// Derivative filtering mode
    pub derivative_mode: DerivativeMode,
    /// Derivative filter coefficient (0.0 to 1.0, typically 0.1-0.3)
    pub derivative_filter_coeff: T,

    /// Maximum allowed time step (seconds) - for safety
    pub max_dt: T,
    /// Minimum allowed time step (seconds) - prevents division issues
    pub min_dt: T,

    /// Enable setpoint ramping (smooth setpoint changes)
    pub setpoint_ramping: bool,
    /// Setpoint ramp rate (units per second)
    pub setpoint_ramp_rate: T,
}

impl<T: Float> Default for PidConfig<T> {
    fn default() -> Self {
        Self {
            kp: T::one(),
            ki: T::zero(),
            kd: T::zero(),
            output_max: T::from(100.0).unwrap(),
            output_min: T::from(-100.0).unwrap(),
            output_rate_limit: None,
            p_limit: T::infinity(),
            i_limit: T::from(100.0).unwrap(),
            d_limit: T::infinity(),
            antiwindup_mode: AntiWindupMode::BackCalculation,
            antiwindup_gain: T::one(),
            derivative_mode: DerivativeMode::LowPass,
            derivative_filter_coeff: T::from(0.2).unwrap(),
            max_dt: T::from(1.0).unwrap(),
            min_dt: T::from(0.0001).unwrap(),
            setpoint_ramping: false,
            setpoint_ramp_rate: T::from(10.0).unwrap(),
        }
    }
}

/// Detailed control output with diagnostics
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ControlOutput<T: Float> {
    /// Final control output
    pub output: T,
    /// Proportional term contribution
    pub p: T,
    /// Integral term contribution
    pub i: T,
    /// Derivative term contribution
    pub d: T,
    /// Current error
    pub error: T,
    /// Setpoint used (after ramping if enabled)
    pub effective_setpoint: T,
    /// Safety status
    pub safety_status: SafetyStatus,
    /// Time delta used for this calculation
    pub dt: T,
}

/// Advanced PID controller for critical systems
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct RobustPid<T: Float> {
    /// Configuration
    config: PidConfig<T>,
    /// Target setpoint
    setpoint: T,
    /// Internal setpoint (after ramping)
    internal_setpoint: T,

    // State variables
    /// Accumulated integral term (already multiplied by ki)
    integral: T,
    /// Previous measurement
    prev_measurement: Option<T>,
    /// Filtered derivative term
    filtered_derivative: T,
    /// Previous output (for rate limiting)
    prev_output: T,
    /// Previous error (for derivative on error, if needed)
    prev_error: Option<T>,

    // Diagnostics
    /// Total number of updates
    update_count: u64,
    /// Number of times output was saturated
    saturation_count: u64,
    /// Maximum observed error magnitude
    max_error: T,
}

impl<T: Float> RobustPid<T> {
    /// Create a new PID controller with the given configuration
    pub fn new(config: PidConfig<T>) -> Self {
        Self {
            internal_setpoint: T::zero(),
            setpoint: T::zero(),
            config,
            integral: T::zero(),
            prev_measurement: None,
            filtered_derivative: T::zero(),
            prev_output: T::zero(),
            prev_error: None,
            update_count: 0,
            saturation_count: 0,
            max_error: T::zero(),
        }
    }

    /// Create a PID controller with default settings
    pub fn with_gains(kp: T, ki: T, kd: T) -> Self {
        let mut config = PidConfig::default();
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
        Self::new(config)
    }

    /// Set the target setpoint
    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint = setpoint;
        if !self.config.setpoint_ramping {
            self.internal_setpoint = setpoint;
        }
    }

    /// Update configuration (allows online tuning)
    pub fn update_config(&mut self, config: PidConfig<T>) {
        // Recalculate integral if ki changed to prevent jumps
        if config.ki != self.config.ki && config.ki != T::zero() {
            self.integral = self.integral * (config.ki / self.config.ki);
        }
        self.config = config;
    }

    /// Get current configuration
    pub fn config(&self) -> &PidConfig<T> {
        &self.config
    }

    /// Compute control output with variable time step
    ///
    /// This method uses setpoint tracking mode: error = setpoint - measurement
    pub fn update(&mut self, measurement: T, dt: T) -> ControlOutput<T> {
        self.update_count += 1;

        // Validate time step
        let dt = self.clamp_dt(dt);

        // Update internal setpoint with ramping if enabled
        if self.config.setpoint_ramping {
            self.update_setpoint_ramp(dt);
        }

        // Calculate error
        let error = self.internal_setpoint - measurement;

        self.update_with_error_and_measurement(error, measurement, dt)
    }

    /// Compute control output directly from error signal
    ///
    /// Use this when you already have the error calculated externally.
    /// Positive error means the system needs to increase output.
    /// Negative error means the system needs to decrease output.
    ///
    /// # Example
    /// ```ignore
    /// // External error source (e.g., from sensor fusion, observer, etc.)
    /// let error = calculate_tracking_error(); // Could be +/- value
    ///
    /// let output = pid.update_from_error(error, dt);
    /// // Controller will drive error toward zero
    /// ```
    pub fn update_from_error(&mut self, error: T, dt: T) -> ControlOutput<T> {
        self.update_count += 1;

        // Validate time step
        let dt = self.clamp_dt(dt);

        // For error-based mode, we don't have a measurement, so we calculate it
        // based on the previous error and the change
        let calculated_measurement = match self.prev_error {
            Some(prev_err) => {
                // If we have previous error, we can estimate the measurement change
                // from the error change (assuming setpoint didn't change)
                match self.prev_measurement {
                    Some(prev_meas) => prev_meas - (error - prev_err),
                    None => T::zero(), // First call, no previous measurement
                }
            }
            None => T::zero(), // First call
        };

        self.update_with_error_and_measurement(error, calculated_measurement, dt)
    }

    /// Internal update logic shared by both update modes
    fn update_with_error_and_measurement(
        &mut self,
        error: T,
        measurement: T,
        dt: T,
    ) -> ControlOutput<T> {
        // Track maximum error for diagnostics
        if error.abs() > self.max_error {
            self.max_error = error.abs();
        }

        // Proportional term
        let p_unbounded = error * self.config.kp;
        let p = self.clamp(p_unbounded, self.config.p_limit);

        // Derivative term (on measurement to avoid derivative kick)
        let d = self.calculate_derivative(measurement, dt);

        // Calculate tentative output before integral
        let tentative_output = p + self.integral + d;

        // Integral term with anti-windup
        let i = self.update_integral(error, dt, tentative_output);

        // Final output calculation
        let mut output = p + i + d;
        output = self.clamp_output(output);

        // Apply output rate limiting if configured
        output = self.apply_rate_limit(output, dt);

        // Determine safety status
        let safety_status = self.determine_safety_status(output, error, dt);

        // Update state
        self.prev_measurement = Some(measurement);
        self.prev_error = Some(error);
        self.prev_output = output;

        ControlOutput {
            output,
            p,
            i,
            d,
            error,
            effective_setpoint: self.internal_setpoint,
            safety_status,
            dt,
        }
    }

    /// Calculate derivative term with filtering
    fn calculate_derivative(&mut self, measurement: T, dt: T) -> T {
        let raw_derivative = match self.prev_measurement {
            Some(prev_measurement) => {
                // Derivative on measurement (prevents derivative kick on setpoint changes)
                -(measurement - prev_measurement) / dt
            }
            None => T::zero(),
        };

        // Apply filtering based on mode
        let derivative = match self.config.derivative_mode {
            DerivativeMode::None => raw_derivative,
            DerivativeMode::LowPass => {
                // First-order low-pass filter: d_filtered = α * d_raw + (1-α) * d_prev
                let alpha = self.config.derivative_filter_coeff;
                alpha * raw_derivative + (T::one() - alpha) * self.filtered_derivative
            }
            DerivativeMode::SimpleMovingAverage => {
                // Simple moving average with fixed alpha
                let alpha = self.config.derivative_filter_coeff;
                alpha * raw_derivative + (T::one() - alpha) * self.filtered_derivative
            }
        };

        self.filtered_derivative = derivative;

        let d_unbounded = derivative * self.config.kd;
        self.clamp(d_unbounded, self.config.d_limit)
    }

    /// Update integral term with anti-windup protection
    fn update_integral(&mut self, error: T, dt: T, tentative_output: T) -> T {
        let error_contribution = error * self.config.ki * dt;

        match self.config.antiwindup_mode {
            AntiWindupMode::Clamping => {
                // Simple clamping of integral
                let tentative_integral = self.integral + error_contribution;
                self.integral = self.clamp(tentative_integral, self.config.i_limit);
            }
            AntiWindupMode::BackCalculation => {
                // Back-calculate: if output would saturate, reduce integral
                let clamped_output = self.clamp_output(tentative_output);
                let output_error = clamped_output - tentative_output;

                // Back-calculation feedback
                let integral_correction = output_error * self.config.antiwindup_gain * dt;
                let tentative_integral = self.integral + error_contribution + integral_correction;
                self.integral = self.clamp(tentative_integral, self.config.i_limit);
            }
            AntiWindupMode::Conditional => {
                // Only integrate if output is not saturated
                let clamped_output = self.clamp_output(tentative_output);
                if (tentative_output - clamped_output).abs() < T::epsilon() {
                    // Not saturated, integrate normally
                    let tentative_integral = self.integral + error_contribution;
                    self.integral = self.clamp(tentative_integral, self.config.i_limit);
                }
                // Otherwise, don't update integral
            }
        }

        self.integral
    }

    /// Apply setpoint ramping for smooth transitions
    fn update_setpoint_ramp(&mut self, dt: T) {
        let error = self.setpoint - self.internal_setpoint;
        let max_change = self.config.setpoint_ramp_rate * dt;

        if error.abs() <= max_change {
            self.internal_setpoint = self.setpoint;
        } else {
            let sign = if error > T::zero() {
                T::one()
            } else {
                -T::one()
            };
            self.internal_setpoint = self.internal_setpoint + sign * max_change;
        }
    }

    /// Apply output rate limiting
    fn apply_rate_limit(&self, output: T, dt: T) -> T {
        if let Some(rate_limit) = self.config.output_rate_limit {
            let max_change = rate_limit * dt;
            let change = output - self.prev_output;

            if change.abs() > max_change {
                let sign = if change > T::zero() {
                    T::one()
                } else {
                    -T::one()
                };
                self.prev_output + sign * max_change
            } else {
                output
            }
        } else {
            output
        }
    }

    /// Clamp value to symmetric limits
    fn clamp(&self, value: T, limit: T) -> T {
        let limit_abs = limit.abs();
        if value > limit_abs {
            limit_abs
        } else if value < -limit_abs {
            -limit_abs
        } else {
            value
        }
    }

    /// Clamp output to configured min/max
    fn clamp_output(&mut self, value: T) -> T {
        let clamped = if value > self.config.output_max {
            self.saturation_count += 1;
            self.config.output_max
        } else if value < self.config.output_min {
            self.saturation_count += 1;
            self.config.output_min
        } else {
            value
        };
        clamped
    }

    /// Validate and clamp time step
    fn clamp_dt(&self, dt: T) -> T {
        if dt < self.config.min_dt {
            self.config.min_dt
        } else if dt > self.config.max_dt {
            self.config.max_dt
        } else {
            dt
        }
    }

    /// Determine safety status based on current state
    fn determine_safety_status(&self, output: T, error: T, dt: T) -> SafetyStatus {
        let mut fault_count = 0;
        let mut status = SafetyStatus::Normal;

        // Check output saturation
        if (output - self.config.output_max).abs() < T::epsilon()
            || (output - self.config.output_min).abs() < T::epsilon()
        {
            status = SafetyStatus::OutputSaturated;
            fault_count += 1;
        }

        // Check integral windup
        if (self.integral.abs() - self.config.i_limit).abs() < T::epsilon() {
            status = SafetyStatus::IntegralWindup;
            fault_count += 1;
        }

        // Check derivative noise (large derivative relative to error)
        if error != T::zero()
            && self.filtered_derivative.abs() > error.abs() * T::from(10.0).unwrap()
        {
            status = SafetyStatus::DerivativeNoisy;
            fault_count += 1;
        }

        // Check time step
        if dt >= self.config.max_dt {
            status = SafetyStatus::TimeStepExcessive;
            fault_count += 1;
        }

        if fault_count > 1 {
            SafetyStatus::MultipleFaults
        } else {
            status
        }
    }

    /// Reset all internal state (use when switching control modes)
    pub fn reset(&mut self) {
        self.integral = T::zero();
        self.prev_measurement = None;
        self.filtered_derivative = T::zero();
        self.prev_output = T::zero();
        self.prev_error = None;
        self.internal_setpoint = self.setpoint;
    }

    /// Reset only integral term (for integral windup recovery)
    pub fn reset_integral(&mut self) {
        self.integral = T::zero();
    }

    /// Manual integral preload (for bumpless transfer)
    pub fn preload_integral(&mut self, value: T) {
        self.integral = self.clamp(value, self.config.i_limit);
    }

    /// Get current integral term value
    pub fn get_integral_term(&self) -> T {
        self.integral
    }

    /// Get diagnostic information
    pub fn diagnostics(&self) -> PidDiagnostics<T> {
        PidDiagnostics {
            update_count: self.update_count,
            saturation_count: self.saturation_count,
            saturation_ratio: if self.update_count > 0 {
                T::from(self.saturation_count).unwrap() / T::from(self.update_count).unwrap()
            } else {
                T::zero()
            },
            max_error: self.max_error,
            current_integral: self.integral,
            current_derivative: self.filtered_derivative,
        }
    }
}

/// Diagnostic information about PID performance
#[derive(Debug, Clone, Copy)]
pub struct PidDiagnostics<T: Float> {
    /// Total number of updates
    pub update_count: u64,
    /// Number of saturations
    pub saturation_count: u64,
    /// Ratio of saturated outputs
    pub saturation_ratio: T,
    /// Maximum error observed
    pub max_error: T,
    /// Current integral value
    pub current_integral: T,
    /// Current derivative value
    pub current_derivative: T,
}

impl<T: Float + fmt::Display> fmt::Display for PidDiagnostics<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PID Diagnostics:\n\
             Updates: {}\n\
             Saturations: {} ({:.2}%)\n\
             Max Error: {}\n\
             Current Integral: {}\n\
             Current Derivative: {}",
            self.update_count,
            self.saturation_count,
            self.saturation_ratio * T::from(100.0).unwrap(),
            self.max_error,
            self.current_integral,
            self.current_derivative
        )
    }
}

// ============================================================================
// Builder Pattern for Easier Configuration
// ============================================================================

pub struct PidBuilder<T: Float> {
    config: PidConfig<T>,
}

impl<T: Float> PidBuilder<T> {
    pub fn new() -> Self {
        Self {
            config: PidConfig::default(),
        }
    }

    pub fn gains(mut self, kp: T, ki: T, kd: T) -> Self {
        self.config.kp = kp;
        self.config.ki = ki;
        self.config.kd = kd;
        self
    }

    pub fn output_limits(mut self, min: T, max: T) -> Self {
        self.config.output_min = min;
        self.config.output_max = max;
        self
    }

    pub fn term_limits(mut self, p_limit: T, i_limit: T, d_limit: T) -> Self {
        self.config.p_limit = p_limit;
        self.config.i_limit = i_limit;
        self.config.d_limit = d_limit;
        self
    }

    pub fn output_rate_limit(mut self, rate: T) -> Self {
        self.config.output_rate_limit = Some(rate);
        self
    }

    pub fn antiwindup(mut self, mode: AntiWindupMode, gain: T) -> Self {
        self.config.antiwindup_mode = mode;
        self.config.antiwindup_gain = gain;
        self
    }

    pub fn derivative_filter(mut self, mode: DerivativeMode, coeff: T) -> Self {
        self.config.derivative_mode = mode;
        self.config.derivative_filter_coeff = coeff;
        self
    }

    pub fn setpoint_ramping(mut self, enabled: bool, rate: T) -> Self {
        self.config.setpoint_ramping = enabled;
        self.config.setpoint_ramp_rate = rate;
        self
    }

    pub fn time_limits(mut self, min_dt: T, max_dt: T) -> Self {
        self.config.min_dt = min_dt;
        self.config.max_dt = max_dt;
        self
    }

    pub fn build(self) -> RobustPid<T> {
        RobustPid::new(self.config)
    }
}

impl<T: Float> Default for PidBuilder<T> {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_proportional_only() {
        let mut pid = PidBuilder::new()
            .gains(2.0, 0.0, 0.0)
            .output_limits(-100.0, 100.0)
            .build();

        pid.set_setpoint(10.0);

        let output = pid.update(5.0, 1.0);
        assert_eq!(output.p, 10.0); // (10-5) * 2.0
        assert_eq!(output.i, 0.0);
        assert_eq!(output.d, 0.0);
        assert_eq!(output.output, 10.0);
    }

    #[test]
    fn test_integral_accumulation() {
        let mut pid = PidBuilder::new()
            .gains(0.0, 1.0, 0.0)
            .output_limits(-100.0, 100.0)
            .build();

        pid.set_setpoint(10.0);

        let output1 = pid.update(8.0, 1.0); // error = 2
        assert_eq!(output1.i, 2.0);

        let output2 = pid.update(8.0, 1.0); // error = 2
        assert_eq!(output2.i, 4.0);
    }

    #[test]
    fn test_output_saturation() {
        let mut pid = PidBuilder::new()
            .gains(10.0, 0.0, 0.0)
            .output_limits(-50.0, 50.0)
            .build();

        pid.set_setpoint(100.0);

        let output = pid.update(0.0, 1.0);
        assert_eq!(output.output, 50.0); // Saturated at max
        assert!(matches!(
            output.safety_status,
            SafetyStatus::OutputSaturated | SafetyStatus::MultipleFaults
        ));
    }
}
