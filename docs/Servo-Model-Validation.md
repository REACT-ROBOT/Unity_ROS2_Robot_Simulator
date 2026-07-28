# Servo Model Validation Report (single pendulum)

Summary of the physical validation of `ServoJointModel` (friction & backlash model) using a single pendulum. The tests live in `Assets/Tests/ServoModelTests/ServoPendulumTests.cs` and can be re-run any time.

## Setup

- Unity 6000.2.7f2 / PhysX ArticulationBody / Fixed Timestep 0.02 s (50 Hz)
- Rig: fixed base + one revolute pendulum
  - mass m = 0.2 kg, COM distance L = 0.2 m (max gravity torque mgL ≈ 0.39 N·m)
- Servo model parameters:

| Parameter | Value |
|---|---|
| Servo PD (Kp, Kd) | 20 N·m/rad, 0.5 N·m/(rad/s) |
| Torque limit | 2 N·m |
| Motor inertia Jm (gear-reflected) | 2×10⁻³ kg·m² |
| Static friction τ_s / Coulomb τ_c | 0.15 / 0.08 N·m |
| Stribeck velocity ω_s / viscous σ_v | 0.1 rad/s / 0.005 N·m/(rad/s) |
| Total gap (2b) | 0.0174 rad (≈1°) |
| Transmission stiffness K / damping D | 400 N·m/rad / 0.5 N·m/(rad/s) |

The animations (GIF) visualize the gap — invisible at true scale (±0.5°) — as a magnified "pin (motor side) in slot (joint side)" schematic. When the pin engages a flank, the contacted wall turns dark blue.

## Unity-rendered footage

Actual in-Unity footage of the model running. The translucent orange arm is the commanded angle; the solid blue arm is the physical joint driven through the servo model. Since a 1° gap is barely visible on camera, an exaggerated 10° demo is also provided.

![servo_demo_preview](images/servo-model/servo_demo_preview.gif)

*(above: excerpt of the exaggerated 10° demo, 2x speed — at the gravity crossing the blue joint falls ahead through the gap; on command reversal only the orange arm moves until the flank re-engages)*

Full videos (mp4, 26 s each, realtime):

- [servo_demo_exag10deg.mp4](videos/servo_demo_exag10deg.mp4) — 10° gap (exaggerated, clearly visible)
- [servo_demo_real1deg.mp4](videos/servo_demo_real1deg.mp4) — 1° gap (the validated real parameters)

Both follow the same choreography: hold → constant-velocity sweep through the bottom (gravity flank hand-over) → triangle-wave reversals (backlash lag) → hold (stiction).

To regenerate (requires a display, no `-nographics`):

```bash
Unity -batchmode -projectPath <this repo> -runTests -testPlatform playmode \
      -testFilter ServoDemoCapture -testResults results.xml
ffmpeg -framerate 25 -i TestOutput/frames_exag10deg/frame_%04d.png \
       -c:v libx264 -pix_fmt yuv420p out.mp4
```

## 1. Hysteresis (basic backlash behavior)

Slow ±0.3 rad triangle command under zero gravity with load-side friction.

![hysteresis_anim](images/servo-model/hysteresis_anim.gif)

On every command reversal the pin traverses the slot before re-engaging the opposite flank (middle) while the joint stands still, tracing the loop in the response plane (right).

![hysteresis](images/servo-model/hysteresis.png)

- The response lags by the gap on every command reversal, forming a lens-shaped hysteresis loop
- **Loop width (at cmd=0): 0.0152 rad** — about 87 % of the nominal 0.0174 rad gap (as expected from the tanh smoothing)

## 2. Gravity crossing (flank hand-over on load-torque reversal)

Constant-velocity sweep −0.6 → +0.6 rad at 0.15 rad/s under gravity. **The command is monotonic**; only the gravity torque changes sign at the bottom. A command-side filter cannot reproduce this by construction.

![gravity_crossing_anim](images/servo-model/gravity_crossing_anim.gif)

As the pendulum passes the bottom (t≈4.3 s) the pin hands over from the left to the right flank (middle) while the command keeps moving; Δ flips sign (top right) and a small "clunk" appears in the joint velocity (bottom right).

![gravity_crossing](images/servo-model/gravity_crossing.png)

- The transmission deflection Δ (middle) **traverses from −0.0045 to +0.0047 rad** at the crossing (tooth flank hand-over)
- A small disturbance appears in the joint velocity (bottom, t≈4.5 s) — the backlash "clunk"
- The actually transmitted torque (`driveForce`) matches gravity exactly (−0.217 N·m while holding) and crosses zero at θ=0

## 3. Friction-velocity curve (Stribeck accuracy)

Constant-velocity tracking without gravity or backlash; friction torque recovered from the steady tracking error (τ_f = Kp·e + Kd·(ω−ωm)).

![friction_curve](images/servo-model/friction_curve.png)

| ω [rad/s] | measured [N·m] | model [N·m] | error |
|---|---|---|---|
| 0.05 | 0.1347 | 0.1348 | −0.0 % |
| 0.10 | 0.1063 | 0.1063 | +0.0 % |
| 0.20 | 0.0822 | 0.0823 | −0.0 % |
| 0.50 | 0.0824 | 0.0825 | −0.1 % |
| 1.00 | 0.0848 | 0.0850 | −0.2 % |
| 2.00 | 0.0897 | 0.0900 | −0.3 % |

The configured Stribeck curve (breakaway peak → Coulomb plateau → viscous rise) is reproduced with **<0.5 % error everywhere**.

## 4. Position hold under gravity (stiction, chatter)

Step command to 0.25 rad held under gravity — the classic stress test for numerical stick-slip chatter.

![stiction_hold_anim](images/servo-model/stiction_hold_anim.gif)

During the transient the pin bounces across the slot; once settled it stays engaged on one flank and comes to a complete rest (with chatter the pin would keep oscillating here).

![stiction_hold](images/servo-model/stiction_hold.png)

- After the transient the joint comes to a complete rest (residual velocity ~10⁻⁷ rad/s, position std ~10⁻¹⁷)
- Steady error 0.0089 rad (≈0.5°) — consistent with stiction (≤τ_s/Kp) + gap + PD gravity sag
- The motor sticks via the Karnopp condition and the holding torque balances gravity exactly through the transmission spring

## Re-running

```bash
Unity -batchmode -nographics -projectPath <this repo> \
      -runTests -testPlatform playmode -testResults results.xml -logFile unity.log
```

Each test writes time-series CSV logs (command, joint angle, motor angle, deflection, actual transmitted torque, …) to `<project>/TestOutput/`.

## Implementation findings (dev notes)

1. **Never fade the transmission stiffness to zero inside the gap**: a flank impact inside one physics step penetrates without resistance and the re-wound spring injects energy (numerical restitution >1). Keep the stiffness constant and encode the backlash in the drive-target anchor (θm − b·tanh(Δ/b))
2. **xDrive units**: target/targetVelocity are in degrees, but stiffness/damping act on the error **converted to radians (N·m/rad)** — confirmed by measuring `driveForce`
3. **Anchor the spring at the predicted end-of-step load position (θl + ωl·dt)**. With a start-of-step anchor, the per-step motion ω·dt exceeds the static deflection τ/K for stiff transmissions and flips the perceived contact flank
4. **Keep actuated ArticulationBodies from sleeping** (`sleepThreshold = 0`): while the motor sticks the drive target never changes, so a slept articulation freezes with a wound-up spring
