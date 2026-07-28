# Servo Model (Friction & Backlash) Guide

This guide explains how to model gearbox friction (Stribeck/static friction) and backlash — both prominent in low-cost servo motors — per joint, configured from the URDF.

## Overview

Adding a `<servo_model>` tag directly under `<robot>` in the URDF attaches a `ServoJointModel` component to the target joint, which simulates:

1. **Virtual motor rotor** — the servo's internal PD controller and gear-reflected inertia are integrated numerically at a high internal rate
2. **Stribeck friction** — motor/gear-side friction with breakaway torque τ_s, Coulomb torque τ_c, Stribeck velocity ω_s and viscous coefficient σ_v. Near zero velocity the Karnopp method reproduces true stick-slip
3. **Backlash** — a continuous dead-zone spring (Modelica ElastoBacklash style) on the relative displacement Δ between motor angle θm and joint angle θl:
   - transmitted torque τ = K·d(Δ) + D·s(Δ)·(ωm − ωl)
   - d(Δ) = Δ − b·tanh(Δ/b)  (b = half of the total gap width)
   - Since the model is a continuous function with no branching, it automatically reproduces not only command reversals but also **load-torque reversals** (e.g. the clunk when an arm crosses vertical)

The transmission spring is realized through the implicit PhysX xDrive, so it remains stable at a 50 Hz physics step even for stiff transmissions.

## URDF syntax

```xml
<robot name="my_robot">
  <joint name="arm_joint" type="revolute">
    <!-- standard URDF: load-side viscous/Coulomb friction (unchanged) -->
    <dynamics damping="0.001" friction="0.02"/>
    ...
  </joint>

  <!-- servo model extension (all units SI: rad, N*m; for prismatic: m, N) -->
  <servo_model joint="arm_joint">
    <friction static="0.12" dynamic="0.08"
              stribeck_velocity="0.1" viscous="0.005"/>
    <backlash width="0.0175" stiffness="400" damping="0.5"/>
    <motor inertia="2e-3" p_gain="20" d_gain="0.5" torque_limit="2.0"/>
  </servo_model>
</robot>
```

Joint commands arriving on the `/joint_states` command topic are routed to the virtual servo controller instead of being written to the xDrive directly.

## Parameter reference

### `<friction>` (motor/gear-side Stribeck friction)

| Attribute | Meaning | Unit | Default |
|---|---|---|---|
| `static` | breakaway torque τ_s | N·m | 0 |
| `dynamic` | Coulomb torque τ_c | N·m | 0 |
| `stribeck_velocity` | Stribeck velocity ω_s | rad/s | 0.1 |
| `viscous` | viscous coefficient σ_v | N·m/(rad/s) | 0 |

Friction curve: τ_f(ω) = τ_c + (τ_s − τ_c)·exp(−(ω/ω_s)²) + σ_v·ω

### `<backlash>` (gap and transmission)

| Attribute | Meaning | Unit | Default |
|---|---|---|---|
| `width` | total gap (dead band 2b) | rad | 0 |
| `stiffness` | engaged transmission stiffness K | N·m/rad | 400 |
| `damping` | engaged transmission damping D | N·m/(rad/s) | 0.5 |

Rule of thumb for `stiffness`: the tooth deflection at maximum torque, τ_max/K, should be roughly 30–50 % of the half gap b.

Note: the dead zone is a smooth tanh-based continuous function, so the flank engagement is gradual and the **observed effective dead band is 20–30 % narrower than the nominal `width`** (about 24 % of the nominal stiffness is already transmitted at Δ = b). Set `width` slightly larger if you want to match a measured backlash.

### `<motor>` (virtual motor)

| Attribute | Meaning | Unit | Default |
|---|---|---|---|
| `inertia` | gear-reflected rotor inertia (gear ratio² × rotor inertia) | kg·m² | 2e-3 |
| `p_gain` | internal servo P gain | N·m/rad | URDF `<drive>` stiffness converted to SI |
| `d_gain` | internal servo D gain | N·m/(rad/s) | URDF `<drive>` damping converted to SI |
| `torque_limit` | motor torque limit | N·m | `<drive>` force_limit |

## Validation

Validation results with plots and numbers are collected in the [Servo Model Validation Report](Servo-Model-Validation.md). The tests live in `Assets/Tests/ServoModelTests/`:

- **Hysteresis** — a slow triangle command under zero gravity with load friction must produce a hysteresis loop of about the gap width in the command-response plane
- **GravityCrossing** — when the pendulum crosses its lowest point (load torque reverses while the command is monotonic), the transmission deflection Δ must traverse from −b to +b
- **FrictionCurve** — the friction-velocity curve recovered from the steady tracking error must match the configured Stribeck curve
- **StictionHold** — holding a position against gravity must not chatter (no stick-slip limit cycle)

## Limitations & notes

- Free in-gap motion faster than the physics step (~20 ms) is smoothed out. Reduce `Fixed Timestep` if you need higher fidelity there
- The `effort` field of `/joint_states` reports the transmitted torque (`driveForce`), which is close to what a real servo would estimate as output torque
- Keep using the standard URDF `<dynamics friction>` (PhysX `jointFriction`) for load-side (joint bearing) friction
