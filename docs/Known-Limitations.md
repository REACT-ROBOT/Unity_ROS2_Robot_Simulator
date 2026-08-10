# Known limitations and deferred work

Things that are **left alone on purpose** rather than left broken, plus the things the design
rules out. The details live in the other documents, so this page only records **what is
deferred and why**, and what you would have to decide before picking it up.

Last updated: 2026-08-10

## Deferred (fixable, but not being fixed now)

### 1. Physical limits of the servo model

`ServoJointModel` carries limits that come from how the model is built and from the physics
engine. The main ones are the ceiling on transmission stiffness (a discrete stability limit,
and below it a quantitative-accuracy limit that bites first) and the engine behaviour where a
joint resting exactly on a limit stops responding to drive torque.

- Detail and measured values: [Servo-Model-Guide.md, "Limitations & notes"](Servo-Model-Guide.md#limitations--notes)
- Validation: [Servo-Model-Validation.md](Servo-Model-Validation.md)

**Why deferred**: neither is a defect in the code. One is inherent to solving in discrete
time, the other is PhysX behaviour. There is no "implement the service" style fix.

**To pick it up** you would be changing the model itself. At minimum, decide:

- whether you want to raise the stiffness ceiling, or to keep it and prove in validation that
  configurations stay under it
- if raising it: solve the transmission implicitly (semi-implicit), or shorten the physics
  step. The first rewrites the model; the second costs whole-scene compute
- whether limit-face sticking should be avoided operationally (stop short of the limit) or
  worked around in code

Re-measure with the diagnostic test in `Assets/Tests/ServoModelTests/ServoEnvironmentProbe.cs`.

### 2. The `humble` branch does not carry the newer checks

The conformance suite's `humble` branch (Unity_ROS2_sample) is a **snapshot of the point
where Humble was verified**. Checks added afterwards — the later H scenarios from `WORLD_TAGS`
onward, the I group, and G6 / F3 / H2b — are not on it.

**Why deferred**: this is what the branch layout decided. `humble` exists for Humble-specific
fixes, not for tracking new features. The checks that are there gate on the advertised
features, so they do not fail against a simulator that has the newer ones.

**To pick it up**: cherry-pick the relevant commits from `main`. Decide first whether both
branches should carry the same checks, or whether `humble` should stay frozen where it is.
See the "Branches" section of the
[Unity_ROS2_sample README](https://github.com/hijimasa/Unity_ROS2_sample).

## Ruled out by design

Accepted as-is; no work planned.

| Item | What | Documented in |
|---|---|---|
| `acceleration` on `set_entity_state` | Readable but ignored when set. Unity has no way to impose an acceleration, and converting to a force means deciding how to treat mass | [Simulation-Interfaces-Services.md](Simulation-Interfaces-Services.md) |
| Acceleration smoothing | A raw first difference of velocity, so contacts make it spike. Smoothing is the caller's job | ditto |
| What counts as an entity | Only what `spawn_entity` / `spawn_entities` created; GUI-placed objects are scenery | ditto |
| Checks a fixed-base robot cannot run | Its base link is `immovable`, so anything assuming the root moves (C5 / H2b) is skipped | [Service-Conformance-Test.md](Service-Conformance-Test.md) |
| Pairing with upstream ROS-TCP-Endpoint | It lacks the unsubscribe and Unity-action system commands, so the two cannot be combined | ditto |
| Wheel slip at speed | The contact point moves too far per physics step for the contact to follow; friction cannot fix it (74 % slip at 1.5 m/s at 50 Hz, 7 % at 200 Hz) | [URDF-Collision-Material.md](URDF-Collision-Material.md) |
| Default friction `combine` | `average`, so the effective value is the mean with the floor's. Use `combine="maximum"` to make the written value hold | ditto |
| `ArticulationBody.jointVelocity` | Does not reflect motion of a joint driven by xDrive; the model uses a finite difference of position instead | [Servo-Model-Guide.md](Servo-Model-Guide.md) |

## simulation_interfaces coverage

Every service in the srv directory, the `SimulateSteps` action, and every flag in
`SimulatorFeatures` are **implemented**. Nothing is outstanding
([Simulation-Interfaces-Services.md](Simulation-Interfaces-Services.md)).
