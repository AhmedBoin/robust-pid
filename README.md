# Robust PID Controller

A production-grade PID controller for safety-critical systems including aerospace, automotive, and industrial control applications.

## Features

- **Anti-windup protection** with 3 strategies (Clamping, Back-calculation, Conditional)
- **Derivative filtering** to reduce noise sensitivity
- **Setpoint ramping** for smooth transitions
- **Output rate limiting** to protect actuators
- **Safety monitoring** with comprehensive status reporting
- **Variable time step** support
- **Bumpless transfer** between control modes
- **Detailed diagnostics** for tuning and analysis
- **`no_std` compatible** for embedded systems
- **Generic over float types** (f32, f64)

## Learn PID Design — Step by Step

If you're new to control systems or want to understand how to design, tune, and analyze a PID controller, check out the complete design guide:

**[Complete PID Controller Design Guide →](https://github.com/AhmedBoin/robust-pid/blob/main/Complete_PID_Controller_Guide.md)**

This document explains:
- The mathematical foundation of PID control  
- Practical tuning techniques (Ziegler–Nichols, Cohen–Coon, etc.)  
- Anti-windup and derivative filtering design  
- Example simulations and tuning workflow  

Perfect for both beginners and professionals who want deeper insight into robust control design.

## Quick Start
```rust
use robust_pid::{PidBuilder, AntiWindupMode, DerivativeMode};

fn main() {
    // Create a PID controller
    let mut pid = PidBuilder::new()
        .gains(2.0, 0.5, 0.1)           // Kp, Ki, Kd
        .output_limits(-100.0, 100.0)    // Output range
        .antiwindup(AntiWindupMode::BackCalculation, 1.0)
        .derivative_filter(DerivativeMode::LowPass, 0.2)
        .build();

    // Set target
    pid.set_setpoint(25.0);

    // Control loop
    let measurement = 20.0;
    let dt = 0.01; // 10ms
    
    let output = pid.update(measurement, dt);
    println!("Control output: {}", output.output);
}
```

## Examples

Run the examples:
```bash
# Simple temperature control
cargo run --example basic_temperature

# Aircraft pitch control
cargo run --example aircraft_pitch

# Rocket 3-axis attitude control
cargo run --example rocket_attitude

# Drone cascaded altitude control
cargo run --example drone_altitude

# Feed with error only
cargo run --example error_correction
```

## no_std Support

For embedded systems without std:
```toml
[dependencies]
robust-pid = { version = "0.1", default-features = false, features = ["libm"] }
```

## License

MIT OR Apache-2.0
