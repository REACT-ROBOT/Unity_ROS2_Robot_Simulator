# Known limitations and deferred work

Things that are **left alone on purpose** rather than left broken, plus the things the design
rules out. The details live in the other documents, so this page only records **what is
deferred and why**, and what you would have to decide before picking it up.

Last updated: 2026-08-11

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

### 3. Robots with water features still gain ~1 kg per collision geometry

When a URDF declares `<buoyancy_material>` or `<hydrodynamics>`, the spawner attaches
`ArticulationFloatingObject` / `HydrodynamicFloatingObject` to each link's collision-geometry
GameObject. Both components carry `[RequireComponent(typeof(ArticulationBody))]`, so Unity
implicitly creates an extra `ArticulationBody` (default mass 1 kg, fixed-jointed to the link)
on every collision geometry. Robots **without** water elements in the URDF no longer receive
these components, so they carry no extra bodies; robots **with** them still do — each collision
geometry adds about 1 kg to the robot's total mass, shifts the link's centre of mass toward the
collider origin, and inflates ground-contact forces and wheel normal loads accordingly.

**Why deferred**: the buoyancy force is computed as `water.Density * (body.mass / density) * g`
from that implicit body's own mass, so zeroing or shrinking the mass would silently change the
floating equilibrium of every existing water robot. The clean fix belongs in the
NaughtyWaterBuoyancy package (referenced by git pin, no local clone): drop the
`RequireComponent(ArticulationBody)`, resolve the body via
`GetComponentInParent<ArticulationBody>()` so forces apply to the link body, and compute
displaced volume from the collider geometry instead of the mass/density ratio.
`HydrodynamicFloatingObject` (in this repo) would need the same change, and its slamming force
also reads `body.mass` directly.

**To pick it up**: patch or fork the package as above, migrate `HydrodynamicFloatingObject` in
step, and re-tune the `density` values of existing water URDFs — today the mass/density ratio
effectively encodes displaced volume relative to the phantom 1 kg body.

## Ruled out by design

Accepted as-is; no work planned.

| Item | What | Documented in |
|---|---|---|
| `acceleration` on `set_entity_state` | Readable but ignored when set. Unity has no way to impose an acceleration, and converting to a force means deciding how to treat mass | [Simulation-Interfaces-Services.md](Simulation-Interfaces-Services.md) |
| Acceleration smoothing | A raw first difference of velocity, so contacts make it spike. Smoothing is the caller's job | ditto |
| What counts as an entity | Only what the spawn path created (`spawn_entity` / `spawn_entities`, or the GUI's URDF button which calls the same implementation); other GUI-placed objects are scenery | ditto |
| Checks a fixed-base robot cannot run | Its base link is `immovable`, so anything assuming the root moves (C5 / H2b) is skipped | [Service-Conformance-Test.md](Service-Conformance-Test.md) |
| Pairing with upstream ROS-TCP-Endpoint | It lacks the unsubscribe and Unity-action system commands, so the two cannot be combined | ditto |
| Wheel slip at speed | The contact point moves too far per physics step for the contact to follow; friction cannot fix it (74 % slip at 1.5 m/s at 50 Hz, 7 % at 200 Hz) | [URDF-Collision-Material.md](URDF-Collision-Material.md) |
| Default friction `combine` | `average`, so the effective value is the mean with the floor's. Use `combine="maximum"` to make the written value hold | ditto |
| `ArticulationBody.jointVelocity` | Does not reflect motion of a joint driven by xDrive; the model uses a finite difference of position instead | [Servo-Model-Guide.md](Servo-Model-Guide.md) |

## simulation_interfaces coverage

Every service in the srv directory, the `SimulateSteps` action, and every flag in
`SimulatorFeatures` are **implemented**. Nothing is outstanding
([Simulation-Interfaces-Services.md](Simulation-Interfaces-Services.md)).
