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

The transmission spring is realized through the implicit PhysX xDrive. Note however that the rotor and the joint exchange state with one physics step of delay, so **the transmission stiffness has a discrete-stability upper bound** (see "Limitations & notes").

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
    <backlash width="0.0175" stiffness="20" damping="0.5"/>
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
| `stiffness` | engaged transmission stiffness K | N·m/rad | 20 |
| `damping` | engaged transmission damping D | N·m/(rad/s) | 0.5 |

`stiffness` is squeezed between two bounds (measured values and the derivation
are in [the validation report](Servo-Model-Validation.md#choosing-the-transmission-stiffness-updated-2026-08-08)):

- **lower**: the tooth deflection τ/K must stay below the half gap b (`K > τ/b`).
  Below that the transmission compliance, not the gap, dominates the deviation
  and there is no backlash left to observe
- **upper**: the dead-zone argument is one step stale with respect to the
  end-of-step load position, which leaves a torque error of **K·ω·dt**. That has
  to be small against the torque you want to resolve — at Δt = 0.02 s, a joint
  moving at ω = 0.15 rad/s and carrying 0.2 N·m wants K ≲ 50

The default of 20 aims at the middle of that range - stiff enough for the gap
to dominate, soft enough to stay inside the accuracy bound. Set it explicitly to
match the real hardware; the bundled servo_demo uses K = 2.

Note also that for robots spawned at runtime (standalone player) the discrete
stability condition √(K·(1/Jm + 1/J_load))·Δt ≲ 1 takes precedence
(Δt = Fixed Timestep). For light loads (J_load ~ 10⁻³ kg·m²) the practical
upper bound is a few N·m/rad; beyond it the joint self-oscillates.

Note: the dead zone is a smooth tanh-based continuous function, so the flank engagement is gradual and the **observed effective dead band is 20–30 % narrower than the nominal `width`** (about 24 % of the nominal stiffness is already transmitted at Δ = b). Set `width` slightly larger if you want to match a measured backlash.

### `<motor>` (virtual motor)

| Attribute | Meaning | Unit | Default |
|---|---|---|---|
| `inertia` | gear-reflected rotor inertia (gear ratio² × rotor inertia) | kg·m² | 2e-3 |
| `p_gain` | internal servo P gain | N·m/rad | raw URDF `<drive>` stiffness value |
| `d_gain` | internal servo D gain | N·m/(rad/s) | raw URDF `<drive>` damping value |
| `torque_limit` | motor torque limit | N·m | raw URDF `<limit effort>` value |

The defaults inherit the **raw numbers** written in the URDF as SI values.
In runtime deployments the `<drive>`/`<limit effort>` entries usually carry
the ×180/π compensation described below, so specify `p_gain`/`d_gain`/
`torque_limit` explicitly in that case.

## Validation

Validation results with plots and numbers are collected in the [Servo Model Validation Report](Servo-Model-Validation.md). The tests live in `Assets/Tests/ServoModelTests/`:

- **Hysteresis** — a slow triangle command under zero gravity with load friction must produce a hysteresis loop of about the gap width in the command-response plane
- **GravityCrossing** — when the pendulum crosses its lowest point (load torque reverses while the command is monotonic), the transmission deflection Δ must traverse from −b to +b
- **FrictionCurve** — the friction-velocity curve recovered from the steady tracking error must match the configured Stribeck curve
- **StictionHold** — holding a position against gravity must not chatter (no stick-slip limit cycle)

## General form of the model (porting to other simulators)

The model is engine-independent and can be implemented in MuJoCo, Gazebo,
etc. from the following coupled system:

**Virtual rotor (reflected to the gear output shaft):**

```
Jm·ω̇m = τ_servo − τ_f(ωm) − τ_t
τ_servo = clip( Kp·(θ_cmd − θm) + Kd·(ω_cmd − ωm), ±τ_max )
τ_f(ω)  = [τ_c + (τ_s − τ_c)·exp(−(ω/ω_s)²)]·sgn(ω) + σ_v·ω   (with Karnopp sticking)
```

**Backlash transmission (rotor → load joint):**

```
Δ    = θm − θl,   b = width/2
d(Δ) = Δ − b·tanh(Δ/b)
s(Δ) = tanh²(Δ/b)
τ_t  = K·d(Δ) + D·max(s(Δ), 0.1)·(ωm − ωl)      ← applied to the load joint
```

The Unity implementation maps τ_t onto the implicit PhysX xDrive
(stiffness = K, target = θl + d(Δ), targetVelocity = ωm), but that is just
one realization.

MuJoCo mapping hints:
- rotor inertia Jm → `armature` (though modelling the rotor as a separate
  1-DOF joint is more faithful when combined with backlash)
- Coulomb/static friction → `frictionloss` (MuJoCo's smoothed friction
  differs from the Stribeck/Karnopp shape; implement the equations above as
  a custom passive force if fidelity matters)
- transmission torque τ_t → a custom passive force between the rotor joint
  and the load joint (`mjcb_passive` etc.); d(Δ) and s(Δ) can be used as-is

## Limitations & notes

- **Two separate upper bounds on the transmission stiffness**: (1) discrete
  stability — the rotor and the load are coupled with one physics step of
  delay, so the coupled mode √(K·(1/Jm + 1/J_load)) × Δt must stay below ~1 or
  the joint self-oscillates (measured at 50 Hz with a light load: K ≈ 200–250).
  Plain xDrive position servos (without servo_model) obey the same bound.
  (2) **quantitative accuracy**, which binds first — the dead-zone argument is
  one step stale, leaving a torque error of K·sech²(Δ/b)·ω·dt. At 50 Hz with
  ω = 0.15 rad/s and 0.2 N·m under test, K ≲ 50 (measured: from K = 100 the
  perceived contact flank inverts)
- **Keep the gravity rest pose away from the joint limits**: a joint that
  comes to rest exactly ON a limit can stop responding to drive torques in
  this engine build, and releasing it by rewriting the drive limits at
  runtime makes the position snap, so no un-jam is attempted. The virtual
  rotor is clamped into the joint limit range so the transmission cannot
  press the joint against a limit indefinitely
- **Editor vs standalone player unit difference**: on the Linux player the
  rotational xDrive stiffness/damping/forceLimit act at π/180 of their
  nominal value (measured from static sag); in the editor they act at face
  value. `ServoJointModel` compensates automatically per environment, but
  for plain joints **write URDF `<drive>`/`<limit effort>` with a ×180/π
  factor for runtime deployments**
- **`ArticulationBody.jointVelocity` is unusable in every environment**: it
  does not report motion an xDrive is producing at all (a plain drive sweeping
  at 0.15 rad/s reads −0.008 rad/s; a statically held joint carries a floor of
  ~0.08 rad/s of garbage). On a joint with no drive it is exact, which is why a
  free-swing check misses it. The model derives the load velocity from a
  position finite difference. **The `velocity` field of `/joint_states` is that
  same engine value, so treat it with care on the control side**
- Free in-gap motion faster than the physics step (~20 ms) is smoothed out. Reduce `Fixed Timestep` if you need higher fidelity there
- The `effort` field of `/joint_states` reports the transmitted torque (`driveForce`), which is close to what a real servo would estimate as output torque
- Keep using the standard URDF `<dynamics friction>` (PhysX `jointFriction`) for load-side (joint bearing) friction
