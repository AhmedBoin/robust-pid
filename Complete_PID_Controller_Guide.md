# Complete Guide to Robust PID Controller

## Table of Contents
1. [Understanding PID Basics](#understanding-pid-basics)
2. [Complete Parameter Explanation](#complete-parameter-explanation)
3. [Step-by-Step: Designing Your PID](#step-by-step-designing-your-pid)
4. [Tuning Methods](#tuning-methods)
5. [Troubleshooting Guide](#troubleshooting-guide)
6. [Advanced: Auto-Tuning & Adaptive Control](#advanced-auto-tuning--adaptive-control)
7. [Real-World Examples](#real-world-examples)

---

## Understanding PID Basics

### What is PID?

A PID controller calculates a control output based on three terms:

```
output = P_term + I_term + D_term

Where:
- P_term = Kp × error              (Proportional: React to current error)
- I_term = Ki × ∫error dt          (Integral: Eliminate steady-state error)
- D_term = Kd × d(error)/dt        (Derivative: Predict future error)
```

### How Each Term Works:

**P (Proportional)**: 
- Provides immediate response proportional to error
- Like pushing harder when you're further from goal
- Too high → Oscillation
- Too low → Slow response

**I (Integral)**:
- Accumulates error over time
- Eliminates steady-state error (persistent offset)
- Like remembering past mistakes
- Too high → Overshoot, instability
- Too low → Never reaches target exactly

**D (Derivative)**:
- Predicts future error based on rate of change
- Dampens oscillations
- Like applying brakes when approaching target
- Too high → Noise amplification
- Too low → Overshooting

---

## Complete Parameter Explanation

Let's break down every single parameter in your builder:

```rust
PidBuilder::new()
    .gains(3.5, 0.8, 1.2)                    // [1]
    .output_limits(-30.0, 30.0)               // [2]
    .term_limits(25.0, 10.0, 15.0)           // [3]
    .antiwindup(AntiWindupMode::BackCalculation, 1.5)  // [4]
    .derivative_filter(DerivativeMode::LowPass, 0.2)   // [5]
    .output_rate_limit(50.0)                 // [6]
    .setpoint_ramping(true, 20.0)            // [7]
    .time_limits(0.001, 0.1)                 // [8]
    .build()
```

### [1] `.gains(Kp, Ki, Kd)`

**Purpose**: The core PID gains that determine controller behavior.

```rust
.gains(3.5, 0.8, 1.2)
//     │    │    └─ Kd: Derivative gain (damping)
//     │    └────── Ki: Integral gain (steady-state correction)
//     └─────────── Kp: Proportional gain (immediate response)
```

**How to choose**:
- Start with: `Kp = 1.0, Ki = 0.0, Kd = 0.0` (P-only)
- Increase Kp until system responds quickly but oscillates slightly
- Add Ki to remove steady-state error (start with Ki = Kp/10)
- Add Kd to reduce oscillation (start with Kd = Kp/3)

**Example values by system type**:
```rust
// Fast, responsive systems (e.g., motor position)
.gains(5.0, 2.0, 0.5)

// Slow, thermal systems (e.g., oven temperature)
.gains(2.0, 0.1, 0.05)

// Medium dynamics (e.g., robot arm)
.gains(3.5, 0.8, 1.2)
```

### [2] `.output_limits(min, max)`

**Purpose**: Clamps the final PID output to physical actuator limits.

```rust
.output_limits(-30.0, 30.0)
//             │      └─ Maximum output (e.g., +30° for servo, +30% throttle)
//             └──────── Minimum output (e.g., -30° for servo, -30% throttle)
```

**How to choose**:
- Set to your actuator's physical limits
- For servo: typically ±45° or ±90°
- For motor: 0-100% or ±100%
- For heater: 0-100%

**Why it matters**:
- Prevents damage to actuators
- Triggers anti-windup protection
- Ensures safety in physical systems

### [3] `.term_limits(p_limit, i_limit, d_limit)`

**Purpose**: Individual limits for each PID term BEFORE they're combined.

```rust
.term_limits(25.0, 10.0, 15.0)
//           │     │     └─ Max absolute value of D term
//           │     └─────── Max absolute value of I term (CRITICAL for anti-windup)
//           └──────────── Max absolute value of P term
```

**How to choose**:

**P limit**: Usually not needed, set to infinity or high value
```rust
p_limit = output_max * 0.8  // Leave room for I and D
```

**I limit** (MOST IMPORTANT): Prevents integral windup
```rust
i_limit = output_max * 0.3 to 0.5  // 30-50% of total output range
```

**D limit**: Prevents derivative spikes from noise
```rust
d_limit = output_max * 0.5  // 50% of total output
```

**Example**:
```rust
// If output_limits are (-100, 100):
.term_limits(80.0, 40.0, 60.0)
//           80% for P, 40% for I, 60% for D
```

### [4] `.antiwindup(mode, gain)`

**Purpose**: Prevents integral term from growing unbounded when output saturates.

```rust
.antiwindup(AntiWindupMode::BackCalculation, 1.5)
//          │                                 └─ Back-calculation gain (Kb)
//          └─────────────────────────────────── Anti-windup strategy
```

**Three Modes**:

**1. Clamping** (Simple):
```rust
.antiwindup(AntiWindupMode::Clamping, 0.0)  // gain ignored
```
- Just stops integrating when integral hits i_limit
- Simple but can cause bumps when switching

**2. BackCalculation** (RECOMMENDED):
```rust
.antiwindup(AntiWindupMode::BackCalculation, 1.0 to 2.0)
```
- Feeds back the difference between actual and desired output
- Smoothly reduces integral when saturated
- Gain 1.0-2.0 typical, higher = more aggressive unwinding

**3. Conditional**:
```rust
.antiwindup(AntiWindupMode::Conditional, 0.0)  // gain ignored
```
- Only integrates when output is NOT saturated
- Good for systems that saturate often

**How to choose gain** (for BackCalculation):
- Start with 1.0
- Increase to 1.5-2.0 if integral unwinds too slowly
- Decrease to 0.5 if system becomes unstable during saturation

### [5] `.derivative_filter(mode, coefficient)`

**Purpose**: Filters derivative term to reduce noise sensitivity.

```rust
.derivative_filter(DerivativeMode::LowPass, 0.2)
//                 │                         └─ Filter coefficient (α)
//                 └─────────────────────────── Filter type
```

**Why needed**: Raw derivative amplifies measurement noise, causing chattering.

**Modes**:

**1. None** (Not recommended):
```rust
.derivative_filter(DerivativeMode::None, 0.0)  // No filtering
```
- Use only with extremely clean sensors
- Will amplify any noise

**2. LowPass** (RECOMMENDED):
```rust
.derivative_filter(DerivativeMode::LowPass, 0.1 to 0.3)
```
- First-order low-pass filter
- Formula: `d_filtered = α × d_raw + (1-α) × d_prev`
- Lower α = more filtering (smoother but slower)
- Higher α = less filtering (faster but noisier)

**3. SimpleMovingAverage**:
```rust
.derivative_filter(DerivativeMode::SimpleMovingAverage, 0.2)
```
- Similar to LowPass but different implementation
- Use for very noisy systems

**How to choose coefficient (α)**:
```rust
// Clean sensors (encoders, potentiometers)
α = 0.3 to 0.5

// Moderate noise (ultrasonic, IR sensors)
α = 0.15 to 0.25

// Very noisy sensors (cheap accelerometers, cameras)
α = 0.05 to 0.15
```

**Rule of thumb**: Start with 0.2, decrease if output is jittery.

### [6] `.output_rate_limit(rate_per_second)`

**Purpose**: Limits how fast the output can change, protecting actuators.

```rust
.output_rate_limit(50.0)  // Maximum change of 50 units per second
```

**How it works**:
```
If current_output = 10, prev_output = 0, dt = 0.1s, rate_limit = 50:
- Desired change = 10 - 0 = 10
- Max allowed change in 0.1s = 50 × 0.1 = 5
- Actual output = 0 + 5 = 5 (limited)
```

**How to choose**:
```rust
// Physical constraint (servo speed): 180°/sec
.output_rate_limit(180.0)

// Safety constraint (gradual acceleration): 10%/sec
.output_rate_limit(10.0)

// Prevent mechanical stress: 30 units/sec
.output_rate_limit(30.0)
```

**When to use**:
- ✅ Mechanical systems (prevents gear damage)
- ✅ Thermal systems (prevents thermal shock)
- ✅ Electrical systems (prevents current spikes)
- ❌ Fast digital systems (unnecessary)

### [7] `.setpoint_ramping(enabled, rate)`

**Purpose**: Smoothly transitions to new setpoints instead of instant jumps.

```rust
.setpoint_ramping(true, 20.0)  // Ramp at 20 units/second
//                │      └─ Ramping rate (units per second)
//                └──────── Enable/disable
```

**Example**:
```rust
// Set target to 100
pid.set_setpoint(100.0);

// With ramping(true, 10.0):
// t=0s:   internal_setpoint = 0
// t=1s:   internal_setpoint = 10  (ramped)
// t=5s:   internal_setpoint = 50  (ramped)
// t=10s:  internal_setpoint = 100 (reached)

// Without ramping:
// t=0s:   internal_setpoint = 100 (instant)
```

**How to choose rate**:
```rust
// Very smooth (passenger comfort - elevator)
.setpoint_ramping(true, 5.0)

// Moderate (industrial robot)
.setpoint_ramping(true, 20.0)

// Fast (racing drone)
.setpoint_ramping(true, 100.0)

// Instant (precision positioning)
.setpoint_ramping(false, 0.0)
```

**Benefits**:
- Prevents derivative kick (D term spike on setpoint change)
- Smoother operation
- Reduces mechanical stress

### [8] `.time_limits(min_dt, max_dt)`

**Purpose**: Safety bounds on time step to prevent numerical issues.

```rust
.time_limits(0.001, 0.1)
//           │      └─ Maximum allowed dt (seconds)
//           └──────── Minimum allowed dt (seconds)
```

**Why needed**:
- **min_dt**: Prevents division by nearly zero in derivative calculation
- **max_dt**: Prevents integration explosion if control loop runs slow

**How to choose**:

**min_dt** (Minimum time step):
```rust
// For 1000 Hz control loop:
.time_limits(0.0001, ...)  // 0.1ms minimum

// For 100 Hz control loop:
.time_limits(0.001, ...)   // 1ms minimum

// Rule: min_dt = 1/(max_expected_frequency × 10)
```

**max_dt** (Maximum time step):
```rust
// For 10 Hz control loop:
.time_limits(..., 0.2)  // 200ms maximum

// For 100 Hz control loop:
.time_limits(..., 0.02) // 20ms maximum

// Rule: max_dt = 2 × expected_dt
```

**Safety trigger**: Sets `SafetyStatus::TimeStepExcessive` if violated.

---

## Step-by-Step: Designing Your PID

### Step 1: Understand Your System

Answer these questions:

1. **What are you controlling?**
   - Position, speed, temperature, pressure, angle?

2. **What are the physical limits?**
   - Actuator range (e.g., servo ±90°, motor 0-100%)

3. **How fast does it respond?**
   - Fast (milliseconds) → motor
   - Medium (seconds) → robot arm
   - Slow (minutes) → oven

4. **Is it noisy?**
   - Clean sensors → less filtering
   - Noisy sensors → more filtering

5. **Safety constraints?**
   - Rate limits? (acceleration)
   - Position limits? (mechanical stops)

### Step 2: Start Simple (P-Only)

```rust
let mut pid = PidBuilder::new()
    .gains(1.0, 0.0, 0.0)  // P-only
    .output_limits(-100.0, 100.0)  // Your actuator limits
    .build();

pid.set_setpoint(target);
```

**Tune Kp**:
1. Start with Kp = 1.0
2. Increase Kp until system responds quickly
3. Keep increasing until you see slight oscillation
4. Reduce Kp by 30-50%

### Step 3: Add Integral (PI Controller)

```rust
.gains(Kp_from_step2, Kp/10.0, 0.0)
.term_limits(100.0, 50.0, 100.0)  // Limit integral
.antiwindup(AntiWindupMode::BackCalculation, 1.0)
```

**Tune Ki**:
1. Start with Ki = Kp/10
2. Increase Ki until steady-state error disappears
3. If overshooting, reduce Ki
4. If too slow, increase Ki slightly

### Step 4: Add Derivative (PID Controller)

```rust
.gains(Kp, Ki, Kp/3.0)
.derivative_filter(DerivativeMode::LowPass, 0.2)
```

**Tune Kd**:
1. Start with Kd = Kp/3
2. Increase Kd to reduce overshoot
3. If output is jittery, reduce Kd or increase filtering

### Step 5: Add Safety Features

```rust
.output_rate_limit(max_safe_rate)  // If needed
.setpoint_ramping(true, ramp_rate)  // If desired
.time_limits(min_dt, max_dt)  // Based on your control rate
```

### Example: Temperature Controller

```rust
// Oven: slow thermal system
let mut pid = PidBuilder::new()
    // Start with P-only, tuned to avoid oscillation
    .gains(2.0, 0.3, 0.1)
    
    // Heater is 0-100%
    .output_limits(0.0, 100.0)
    
    // Integral limited to prevent windup during heat-up
    .term_limits(80.0, 30.0, 20.0)
    
    // Back-calculation to handle long saturation periods
    .antiwindup(AntiWindupMode::BackCalculation, 1.0)
    
    // Strong filtering due to noisy thermocouple
    .derivative_filter(DerivativeMode::LowPass, 0.1)
    
    // Gentle heating to prevent thermal shock
    .output_rate_limit(10.0)  // 10%/sec
    
    // Smooth temperature changes
    .setpoint_ramping(true, 5.0)  // 5°C/sec
    
    // Control loop runs at 1 Hz (slow)
    .time_limits(0.1, 2.0)
    
    .build();
```

### Example: Drone Altitude Controller

```rust
// Fast, responsive system
let mut pid = PidBuilder::new()
    // Aggressive gains for quick response
    .gains(2.5, 0.8, 0.3)
    
    // Throttle 0-100%
    .output_limits(0.0, 100.0)
    
    // Large integral limit for wind compensation
    .term_limits(80.0, 40.0, 30.0)
    
    // Back-calculation for windy conditions
    .antiwindup(AntiWindupMode::BackCalculation, 1.5)
    
    // Light filtering (good sensors)
    .derivative_filter(DerivativeMode::LowPass, 0.25)
    
    // Fast throttle changes allowed
    .output_rate_limit(200.0)
    
    // No ramping (precision control)
    .setpoint_ramping(false, 0.0)
    
    // High-speed loop at 50 Hz
    .time_limits(0.005, 0.05)
    
    .build();
```

---

## Tuning Methods

### Method 1: Manual Tuning (Recommended for beginners)

**Steps**:
1. Set Ki = 0, Kd = 0
2. Increase Kp until oscillation
3. Reduce Kp to 50-70% of that value
4. Set Ki = Kp/10, adjust to eliminate steady-state error
5. Set Kd = Kp/3, adjust to reduce overshoot

### Method 2: Ziegler-Nichols

**Ultimate Gain Method**:
1. Set Ki = 0, Kd = 0
2. Increase Kp until system oscillates constantly
3. Note: Ku (ultimate gain) and Tu (oscillation period)

```rust
Kp = 0.6 × Ku
Ki = 2 × Kp / Tu
Kd = Kp × Tu / 8

// Example: If Ku = 5.0, Tu = 2.0 seconds:
.gains(3.0, 3.0, 0.75)
```

**Note**: Often produces aggressive control, reduce all gains by 30-50% for smoother operation.

### Method 3: Cohen-Coon (For systems with delay)

Good for thermal systems, process control.

1. Run open-loop step test
2. Measure: delay time, rise time, steady-state gain
3. Use lookup tables (see Wikipedia: Cohen-Coon)

### Method 4: Trial and Error with Diagnostics

```rust
loop {
    let output = pid.update(measurement, dt);
    
    // Check diagnostics
    let diag = pid.diagnostics();
    
    if diag.saturation_ratio > 0.5 {
        println!("Too much saturation! Reduce gains or increase limits");
    }
    
    if output.safety_status == SafetyStatus::DerivativeNoisy {
        println!("Derivative too noisy! Increase filtering");
    }
}
```

---

## Troubleshooting Guide

### Problem: System Oscillates

**Symptoms**: Output swings back and forth around setpoint

**Solutions**:
```rust
// 1. Reduce ALL gains
.gains(Kp * 0.5, Ki * 0.5, Kd * 0.5)

// 2. Reduce P gain specifically
.gains(Kp * 0.7, Ki, Kd)

// 3. Increase D gain and filtering
.gains(Kp, Ki, Kd * 1.5)
.derivative_filter(DerivativeMode::LowPass, 0.15)  // More filtering

// 4. Add output rate limiting
.output_rate_limit(30.0)
```

### Problem: Slow Response

**Symptoms**: Takes too long to reach setpoint

**Solutions**:
```rust
// 1. Increase P gain
.gains(Kp * 1.5, Ki, Kd)

// 2. Increase I gain (if steady-state error exists)
.gains(Kp, Ki * 1.3, Kd)

// 3. Add setpoint ramping (counterintuitive but prevents overshoot)
.setpoint_ramping(true, 10.0)

// 4. Check if saturating too much
.term_limits(p_lim * 1.5, i_lim * 1.5, d_lim)
```

### Problem: Overshooting

**Symptoms**: Goes past setpoint before settling

**Solutions**:
```rust
// 1. Reduce I gain
.gains(Kp, Ki * 0.5, Kd)

// 2. Increase D gain
.gains(Kp, Ki, Kd * 1.5)

// 3. Add output rate limiting
.output_rate_limit(40.0)

// 4. Add setpoint ramping
.setpoint_ramping(true, 15.0)
```

### Problem: Steady-State Error

**Symptoms**: Never quite reaches setpoint

**Solutions**:
```rust
// 1. Increase I gain
.gains(Kp, Ki * 1.5, Kd)

// 2. Increase integral limit
.term_limits(p_lim, i_lim * 2.0, d_lim)

// 3. Check for constant disturbance
// The integral should handle it, but may need more time

// 4. Verify no integral windup
pid.diagnostics()  // Check if integral is saturated
```

### Problem: Noisy/Jittery Output

**Symptoms**: Output jumps around rapidly

**Solutions**:
```rust
// 1. Reduce D gain
.gains(Kp, Ki, Kd * 0.5)

// 2. Increase derivative filtering
.derivative_filter(DerivativeMode::LowPass, 0.1)  // More filtering

// 3. Add output rate limiting
.output_rate_limit(50.0)

// 4. Check measurement noise
// Consider filtering measurements before PID
```

### Problem: Integral Windup

**Symptoms**: System overshoots badly after saturation

**Solutions**:
```rust
// 1. Reduce integral limit
.term_limits(p_lim, i_lim * 0.5, d_lim)

// 2. Use stronger anti-windup
.antiwindup(AntiWindupMode::BackCalculation, 2.0)  // Higher gain

// 3. Use conditional integration
.antiwindup(AntiWindupMode::Conditional, 0.0)

// 4. Manually reset if needed
if system_saturated_too_long {
    pid.reset_integral();
}
```

---

## Advanced: Auto-Tuning & Adaptive Control

### Current Library Support

The library does NOT have built-in auto-tuning, but you can implement it on top. Here's how:

### 1. Online Parameter Adaptation

```rust
struct AdaptivePid {
    pid: RobustPid<f64>,
    performance_monitor: PerformanceMonitor,
}

struct PerformanceMonitor {
    error_history: Vec<f64>,
    output_history: Vec<f64>,
    oscillation_count: usize,
    settling_time: f64,
}

impl AdaptivePid {
    fn adaptive_update(&mut self, measurement: f64, dt: f64) -> f64 {
        let output = self.pid.update(measurement, dt);
        
        // Monitor performance
        self.performance_monitor.update(output.error, output.output);
        
        // Adapt every 100 samples
        if self.performance_monitor.sample_count() % 100 == 0 {
            self.adapt_gains();
        }
        
        output.output
    }
    
    fn adapt_gains(&mut self) {
        let metrics = self.performance_monitor.calculate_metrics();
        
        // If oscillating too much, reduce gains
        if metrics.oscillation_ratio > 0.3 {
            let config = self.pid.config().clone();
            let mut new_config = config;
            new_config.kp *= 0.9;
            new_config.ki *= 0.9;
            new_config.kd *= 0.9;
            self.pid.update_config(new_config);
            println!("Reduced gains due to oscillation");
        }
        
        // If too slow, increase proportional
        if metrics.settling_time > 5.0 {
            let mut config = self.pid.config().clone();
            config.kp *= 1.1;
            self.pid.update_config(config);
            println!("Increased Kp for faster response");
        }
    }
}
```

### 2. Relay Auto-Tuning (Åström-Hägglund)

Most common auto-tuning method:

```rust
struct RelayAutoTuner {
    state: TuningState,
    relay_amplitude: f64,
    oscillation_data: Vec<(f64, f64)>,  // (time, measurement)
}

enum TuningState {
    Idle,
    Exciting,  // Applying relay
    Analyzing,
    Complete,
}

impl RelayAutoTuner {
    fn step(&mut self, measurement: f64, setpoint: f64, dt: f64) -> Option<(f64, f64, f64)> {
        match self.state {
            TuningState::Exciting => {
                // Apply relay control
                let error = setpoint - measurement;
                let output = if error > 0.0 {
                    self.relay_amplitude
                } else {
                    -self.relay_amplitude
                };
                
                self.oscillation_data.push((dt, measurement));
                
                // Detect when we have enough oscillations
                if self.has_sufficient_data() {
                    self.state = TuningState::Analyzing;
                }
                
                None
            }
            TuningState::Analyzing => {
                // Calculate ultimate gain and period
                let (ku, tu) = self.calculate_ultimate_params();
                
                // Apply Ziegler-Nichols rules
                let kp = 0.6 * ku;
                let ki = 2.0 * kp / tu;
                let kd = kp * tu / 8.0;
                
                self.state = TuningState::Complete;
                Some((kp, ki, kd))
            }
            _ => None,
        }
    }
}

// Usage:
let mut tuner = RelayAutoTuner::new(10.0);  // Relay amplitude
let mut pid = PidBuilder::new().gains(0.0, 0.0, 0.0).build();

// Run tuning phase
loop {
    let measurement = read_sensor();
    
    if let Some((kp, ki, kd)) = tuner.step(measurement, target, dt) {
        // Tuning complete, update PID
        let mut config = pid.config().clone();
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
        pid.update_config(config);
        println!("Auto-tuning complete: Kp={}, Ki={}, Kd={}", kp, ki, kd);
        break;
    }
}

// Now use normal PID control
```

### 3. Machine Learning / Reinforcement Learning

You can use RL to find optimal gains:

```rust
// Pseudo-code for RL-based tuning

struct RLTuner {
    q_table: HashMap<State, HashMap<Action, f64>>,
    learning_rate: f64,
    exploration_rate: f64,
}

struct State {
    error_magnitude: Bucket,
    error_rate: Bucket,
    integral_size: Bucket,
}

enum Action {
    IncreaseKp,
    DecreaseKp,
    IncreaseKi,
    DecreaseKi,
    IncreaseKd,
    DecreaseKd,
    NoChange,
}

impl RLTuner {
    fn update(&mut self, state: State, reward: f64) {
        // Q-learning update
        // Reward based on: error reduction, smoothness, settling time
        
        let action = self.choose_action(&state);
        // Update Q-table
        // Apply action to PID gains
    }
}

// Reward function
fn calculate_reward(error: f64, output_change: f64, settling: bool) -> f64 {
    let mut reward = 0.0;
    
    // Penalize large errors
    reward -= error.abs() * 10.0;
    
    // Penalize large output changes (smoothness)
    reward -= output_change.abs();
    
    // Reward reaching setpoint
    if error.abs() < 0.1 {
        reward += 100.0;
    }
    
    // Penalize oscillation
    if output_change.abs() > threshold {
        reward -= 50.0;
    }
    
    reward
}
```

### 4. Model-Based Adaptive Control

For systems with known dynamics:

```rust
struct ModelBasedController {
    pid: RobustPid<f64>,
    system_model: SystemModel,
    gain_scheduler: GainScheduler,
}

impl ModelBasedController {
    fn update(&mut self, measurement: f64, operating_point: f64, dt: f64) -> f64 {
        // Calculate optimal gains based on current operating point
        let (kp, ki, kd) = self.gain_scheduler.compute_gains(operating_point);
        
        // Update PID gains
        let mut config = self.pid.config().clone();
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
        self.pid.update_config(config);
        
        // Run PID
        let output = self.pid.update(measurement, dt);
        output.output
    }
}
```

### 5. Safe Online Learning

To adapt without affecting the running process:

```rust
struct SafeAdaptiveController {
    primary_pid: RobustPid<f64>,
    shadow_pid: RobustPid<f64>,  // Testing new gains
    learning_enabled: bool,
    safety_monitor: SafetyMonitor,
}

impl SafeAdaptiveController {
    fn update(&mut self, measurement: f64, dt: f64) -> f64 {
        // Always use primary PID for actual control
        let primary_output = self.primary_pid.update(measurement, dt);
        
        if self.learning_enabled {
            // Run shadow PID in parallel (doesn't affect system)
            let shadow_output = self.shadow_pid.update(measurement, dt);
            
            // Compare performance
            let shadow_better = self.evaluate_performance(
                primary_output.error,
                shadow_output.error
            );
            
            if shadow_better && self.safety_monitor.is_safe() {
                // Gradually migrate to new gains
                self.blend_gains(0.1);  // 10% blend per update
            }
        }
        
        // Always return primary output for safety
        primary_output.output
    }
    
    fn blend_gains(&mut self, alpha: f64) {
        let primary_config = self.primary_pid.config();
        let shadow_config = self.shadow_pid.config();
        
        let mut new_config = primary_config.clone();
        new_config.kp = primary_config.kp * (1.0 - alpha) + shadow_config.kp * alpha;
        new_config.ki = primary_config.ki * (1.0 - alpha) + shadow_config.ki * alpha;
        new_config.kd = primary_config.kd * (1.0 - alpha) + shadow_config.kd * alpha;
        
        self.primary_pid.update_config(new_config);
    }
    
    // Enable/disable learning at runtime
    fn set_learning(&mut self, enabled: bool) {
        self.learning_enabled = enabled;
        if enabled {
            println!("Learning ENABLED - shadow PID active");
        } else {
            println!("Learning DISABLED - using primary PID only");
        }
    }
}

struct SafetyMonitor {
    max_error: f64,
    max_output_change: f64,
}

impl SafetyMonitor {
    fn is_safe(&self) -> bool {
        // Check if it's safe to switch gains
        // - Error not too large
        // - Output not changing too rapidly
        // - No safety faults detected
        true  // Implement your safety logic
    }
}

// Usage:
let mut controller = SafeAdaptiveController::new();

// Start with learning disabled for safety
controller.set_learning(false);

// After system is stable, enable learning
if system_stable() {
    controller.set_learning(true);
}

// Can disable anytime if issues detected
if emergency_condition() {
    controller.set_learning(false);
}
```

### 6. Particle Swarm Optimization (PSO) for Offline Tuning

Use PSO to find optimal gains in simulation:

```rust
struct Particle {
    position: PidGains,  // (Kp, Ki, Kd)
    velocity: PidGains,
    best_position: PidGains,
    best_fitness: f64,
}

struct PidGains {
    kp: f64,
    ki: f64,
    kd: f64,
}

struct PSOTuner {
    particles: Vec<Particle>,
    global_best: PidGains,
    global_best_fitness: f64,
}

impl PSOTuner {
    fn optimize(&mut self, simulator: &mut SystemSimulator) -> PidGains {
        for iteration in 0..100 {
            for particle in &mut self.particles {
                // Evaluate fitness
                let fitness = self.evaluate_fitness(&particle.position, simulator);
                
                // Update personal best
                if fitness > particle.best_fitness {
                    particle.best_position = particle.position.clone();
                    particle.best_fitness = fitness;
                }
                
                // Update global best
                if fitness > self.global_best_fitness {
                    self.global_best = particle.position.clone();
                    self.global_best_fitness = fitness;
                }
            }
            
            // Update velocities and positions
            self.update_swarm();
        }
        
        self.global_best.clone()
    }
    
    fn evaluate_fitness(&self, gains: &PidGains, sim: &mut SystemSimulator) -> f64 {
        // Run simulation with these gains
        let mut pid = PidBuilder::new()
            .gains(gains.kp, gains.ki, gains.kd)
            .build();
        
        sim.reset();
        let mut total_error = 0.0;
        let mut settling_time = 0.0;
        let mut overshoot = 0.0;
        
        for step in 0..1000 {
            let measurement = sim.get_measurement();
            let output = pid.update(measurement, 0.01);
            sim.apply_control(output.output);
            
            total_error += output.error.abs();
            // Track overshoot, settling time, etc.
        }
        
        // Fitness function (higher is better)
        let fitness = -total_error - overshoot * 100.0 - settling_time * 10.0;
        fitness
    }
}
```

---

## Real-World Examples

### Example 1: Quadcopter Altitude Hold

```rust
// System characteristics:
// - Fast dynamics (responds in ~0.1s)
// - Noisy barometer sensor
// - Wind disturbances
// - Control loop: 50 Hz

let mut altitude_pid = PidBuilder::new()
    // Aggressive for quick response to user commands
    .gains(2.5, 0.8, 0.3)
    
    // Throttle range (0-100%)
    .output_limits(0.0, 100.0)
    
    // Integral handles wind, but limit to prevent runaway
    .term_limits(80.0, 30.0, 25.0)
    
    // Back-calculation for sustained wind
    .antiwindup(AntiWindupMode::BackCalculation, 1.5)
    
    // Moderate filtering (barometer is somewhat noisy)
    .derivative_filter(DerivativeMode::LowPass, 0.2)
    
    // Allow fast throttle changes
    .output_rate_limit(200.0)
    
    // No ramping (user expects instant response)
    .setpoint_ramping(false, 0.0)
    
    // 50 Hz control
    .time_limits(0.01, 0.05)
    
    .build();

altitude_pid.set_setpoint(5.0);  // 5 meters altitude

loop {
    let altitude = barometer.read();
    let dt = timer.elapsed();
    
    let output = altitude_pid.update(altitude, dt);
    
    // Check safety
    match output.safety_status {
        SafetyStatus::OutputSaturated => {
            // Normal during rapid ascent/descent
        }
        SafetyStatus::DerivativeNoisy => {
            println!("Warning: Noisy derivative, check sensor");
        }
        SafetyStatus::MultipleFaults => {
            println!("ERROR: Multiple faults!");
            emergency_land();
        }
        _ => {}
    }
    
    motors.set_throttle(output.output);
    
    sleep(dt);
}
```

### Example 2: Industrial Oven Temperature Control

```rust
// System characteristics:
// - Very slow (thermal time constant ~5 minutes)
// - Noisy thermocouple
// - Large thermal mass
// - Control loop: 1 Hz

let mut temp_pid = PidBuilder::new()
    // Conservative gains for slow thermal system
    .gains(2.0, 0.1, 0.05)
    
    // Heater power 0-100%
    .output_limits(0.0, 100.0)
    
    // Small integral to handle heat loss
    .term_limits(80.0, 20.0, 15.0)
    
    // Conditional integration (don't integrate during heat-up)
    .antiwindup(AntiWindupMode::Conditional, 0.0)
    
    // Strong filtering for noisy thermocouple
    .derivative_filter(DerivativeMode::LowPass, 0.05)
    
    // Slow power changes to prevent thermal shock
    .output_rate_limit(5.0)  // 5%/second
    
    // Smooth temperature profile
    .setpoint_ramping(true, 2.0)  // 2°C/second
    
    // Slow 1 Hz control
    .time_limits(0.5, 2.0)
    
    .build();

temp_pid.set_setpoint(250.0);  // 250°C target

loop {
    let temperature = thermocouple.read();
    let dt = 1.0;  // 1 second
    
    let output = temp_pid.update(temperature, dt);
    
    // Safety checks
    if temperature > 300.0 {
        heater.set_power(0.0);
        panic!("Overheat!");
    }
    
    heater.set_power(output.output);
    
    // Log diagnostics every minute
    if frame % 60 == 0 {
        println!("{}", temp_pid.diagnostics());
    }
    
    sleep(Duration::from_secs(1));
}
```

### Example 3: CNC Machine Axis Control

```rust
// System characteristics:
// - Medium-fast (responds in ~0.05s)
// - High precision encoders (very clean)
// - Needs smooth motion
// - Control loop: 200 Hz

let mut position_pid = PidBuilder::new()
    // Moderate gains for precision
    .gains(3.0, 0.5, 1.0)
    
    // Motor velocity command (mm/s)
    .output_limits(-500.0, 500.0)
    
    // Tight limits for precision
    .term_limits(400.0, 100.0, 150.0)
    
    // Back-calculation
    .antiwindup(AntiWindupMode::BackCalculation, 1.0)
    
    // Minimal filtering (encoder is clean)
    .derivative_filter(DerivativeMode::LowPass, 0.4)
    
    // Smooth acceleration
    .output_rate_limit(1000.0)  // 1000 mm/s²
    
    // Smooth motion profile
    .setpoint_ramping(true, 100.0)  // 100 mm/s max speed
    
    // 200 Hz control
    .time_limits(0.001, 0.01)
    
    .build();

// Cutting a path
let path = vec![0.0, 10.0, 20.0, 30.0, 40.0];

for target_position in path {
    position_pid.set_setpoint(target_position);
    
    while !at_target() {
        let position = encoder.read_position();
        let dt = 0.005;  // 200 Hz
        
        let output = position_pid.update(position, dt);
        
        motor.set_velocity(output.output);
        
        // Check for position error
        if output.error.abs() > 1.0 {
            println!("Warning: Following error too large");
        }
        
        sleep_micros(5000);  // 5ms
    }
}
```

### Example 4: Cruise Control (Car)

```rust
// System characteristics:
// - Slow (car acceleration takes seconds)
// - Hills create disturbances
// - Smooth acceleration required
// - Control loop: 10 Hz

let mut speed_pid = PidBuilder::new()
    // Gentle gains for passenger comfort
    .gains(1.5, 0.3, 0.2)
    
    // Throttle 0-100%
    .output_limits(0.0, 100.0)
    
    // Integral handles hills
    .term_limits(80.0, 40.0, 30.0)
    
    // Back-calculation for steep hills
    .antiwindup(AntiWindupMode::BackCalculation, 1.0)
    
    // Light filtering (speed sensor is clean)
    .derivative_filter(DerivativeMode::LowPass, 0.3)
    
    // Smooth acceleration for comfort
    .output_rate_limit(10.0)  // 10%/second
    
    // Smooth speed changes
    .setpoint_ramping(true, 5.0)  // 5 km/h per second
    
    // 10 Hz control
    .time_limits(0.05, 0.2)
    
    .build();

speed_pid.set_setpoint(100.0);  // 100 km/h

loop {
    let current_speed = speedometer.read();
    let dt = 0.1;  // 10 Hz
    
    let output = speed_pid.update(current_speed, dt);
    
    throttle.set_position(output.output);
    
    // User can adjust cruise speed
    if cruise_up_button_pressed() {
        let new_speed = speed_pid.config().output_max.min(current_speed + 5.0);
        speed_pid.set_setpoint(new_speed);
    }
    
    sleep(Duration::from_millis(100));
}
```

### Example 5: Error-Based Robot Joint Correction

```rust
// Using direct error input for external error calculation
// (e.g., from vision system detecting misalignment)

let mut correction_pid = PidBuilder::new()
    .gains(1.5, 0.2, 0.3)
    .output_limits(-45.0, 45.0)  // Joint angle correction
    .term_limits(40.0, 15.0, 25.0)
    .antiwindup(AntiWindupMode::BackCalculation, 1.0)
    .derivative_filter(DerivativeMode::LowPass, 0.25)
    .build();

loop {
    // Vision system calculates alignment error
    let alignment_error = vision_system.calculate_error();
    // Positive error = need to move right
    // Negative error = need to move left
    
    let dt = 0.02;  // 50 Hz
    
    // Feed error directly
    let output = correction_pid.update_from_error(alignment_error, dt);
    
    // Apply correction
    robot_joint.apply_correction(output.output);
    
    // The error should reduce over time as corrections are applied
    if alignment_error.abs() < 0.1 {
        println!("Aligned!");
        break;
    }
    
    sleep(Duration::from_millis(20));
}
```

---

## Quick Reference Table

| System Type | Kp | Ki | Kd | Filter α | Rate Limit | Ramping |
|-------------|----|----|----|---------:|------------|---------|
| **Motor Position** | 5.0 | 2.0 | 0.5 | 0.3 | High | No |
| **Motor Speed** | 3.0 | 1.0 | 0.2 | 0.3 | High | Optional |
| **Temperature** | 2.0 | 0.1 | 0.05 | 0.1 | Low | Yes |
| **Pressure** | 1.5 | 0.2 | 0.1 | 0.15 | Medium | Yes |
| **Drone Altitude** | 2.5 | 0.8 | 0.3 | 0.2 | High | No |
| **Robot Arm** | 3.0 | 0.5 | 1.0 | 0.25 | Medium | Yes |
| **Cruise Control** | 1.5 | 0.3 | 0.2 | 0.3 | Low | Yes |
| **Level Control** | 2.0 | 0.4 | 0.15 | 0.2 | Medium | Yes |

---

## Summary Checklist

When designing a PID controller:

✅ **Understand your system**
- [ ] Know the time constant (how fast it responds)
- [ ] Know the physical limits (actuator range)
- [ ] Know the noise characteristics (sensor quality)

✅ **Start simple**
- [ ] Begin with P-only control
- [ ] Add I only if steady-state error exists
- [ ] Add D only if oscillation is a problem

✅ **Set safety limits**
- [ ] output_limits = actuator physical range
- [ ] i_limit = 30-50% of output range
- [ ] max_dt = 2× expected time step

✅ **Configure anti-windup**
- [ ] Use BackCalculation for most cases
- [ ] Set gain to 1.0-2.0
- [ ] Use Conditional if saturating often

✅ **Filter the derivative**
- [ ] Use LowPass mode
- [ ] α = 0.2 for moderate noise
- [ ] Decrease α if output is jittery

✅ **Add safety features**
- [ ] output_rate_limit for mechanical protection
- [ ] setpoint_ramping for smooth operation
- [ ] Monitor safety_status in your control loop

✅ **Tune and test**
- [ ] Start with manual tuning
- [ ] Use diagnostics to identify issues
- [ ] Test with disturbances and edge cases

✅ **Advanced: Adaptive tuning**
- [ ] Use shadow PID for safe online learning
- [ ] Monitor performance metrics
- [ ] Allow enable/disable of learning at runtime
- [ ] Always prioritize safety over performance

---

## Final Notes

**The library does NOT include auto-tuning** because:
1. Every system is different
2. Safety-critical systems need human validation
3. Gives you full control over the tuning process

**But you CAN add it** using the patterns shown above:
- Online adaptation with shadow PID
- Relay auto-tuning
- Machine learning (RL, PSO)
- Model-based scheduling

**Most important**: Start simple, tune manually first, then add complexity only if needed. A well-tuned PID is often better than a poorly-tuned adaptive controller!

**For production systems**: Always have:
- Manual override capability
- Safety monitoring
- Diagnostic logging
- Ability to freeze learning during critical operations