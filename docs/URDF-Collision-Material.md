# collision_material — friction and sensor-only colliders from URDF

URDF has no standard element for contact friction, nor for "this shape is seen by sensors
but does not push back", so this simulator reads a custom `<collision_material>`. The URDF
Importer does not know the element, so the simulator applies it after import
(`Packages/UrdfProperties/Runtime/CollisionMaterialApplier.cs`).

## Syntax

Definitions go directly under `<robot>` and are referenced by name from each `<collision>`.

```xml
<robot name="diffbot">
  <collision_material name="wheel">
    <friction static="1.0" dynamic="1.0" combine="maximum"/>
    <contact_offset value="0.02"/>
  </collision_material>

  <link name="left_wheel_link">
    <collision>
      <geometry><cylinder radius="0.05" length="0.02"/></geometry>
      <collision_material name="wheel"/>
    </collision>
  </link>
</robot>
```

| Element / attribute | Meaning | Default |
|---|---|---|
| `friction@static` | Static friction coefficient | 0 |
| `friction@dynamic` | Dynamic friction coefficient | 0 |
| `friction@combine` | How to combine with the other surface (`average` / `minimum` / `multiply` / `maximum`) | `average` |
| `contact_offset@value` | Collider contact offset [m] | left alone (Unity's 0.01) |
| `sensor_only@value` | `true` makes the collider a trigger: it is hit by raycast sensors (lidar, depth camera) but produces no contact. The element alone means `true` | not a trigger |
| `magnetic_tape@polarity` | marks the shape as magnetic tape for the magnetic guide sensor (`track` or `marker`); implies `sensor_only`. See [Magnetic-Guide-Sensor.md](Magnetic-Guide-Sensor.md) | not tape |

With several `<collision>` elements on one link, **the i-th `<collision>` maps to the i-th
shape**. If one `<collision>` expands into several colliders (submeshes, say), all of them
get the material.

The old name `<physics_material>` still parses, with a warning.

## Sensor-only objects (weeds, tall grass, hanging cloth)

Some things must show up in a lidar scan without stopping the robot. A link without
`<collision>` does **not** do this: the lidar is a physics raycast and only sees colliders,
so a collision-less link is simply invisible. Declare the shapes as normal `<collision>`
elements and mark them `sensor_only` instead:

```xml
<robot name="weeds">
  <collision_material name="weed">
    <sensor_only value="true"/>
  </collision_material>

  <link name="world"/>                      <!-- keeps the entity in place -->
  <link name="weeds_link">
    <collision>
      <origin xyz="1.0 0.2 0.15"/>
      <geometry><cylinder radius="0.05" length="0.3"/></geometry>
      <collision_material name="weed"/>
    </collision>
    <collision>
      <origin xyz="1.3 -0.1 0.15"/>
      <geometry><cylinder radius="0.04" length="0.3"/></geometry>
      <collision_material name="weed"/>
    </collision>
  </link>
  <joint name="fix" type="fixed"><parent link="world"/><child link="weeds_link"/></joint>
</robot>
```

What this does, and what to keep in mind:

- `sensor_only` sets Unity's `Collider.isTrigger`. Triggers take part in raycasts (the project
  has *Queries Hit Triggers* on) but never in contact resolution, so the robot drives through
  them and `get_contact_events` does not record them.
- **Put one link with many `<collision>` shapes**, not one link per plant. Every link is an
  articulation body and PhysX caps an articulation at 64 bodies; colliders per link are not
  limited.
- **Hang the link from a `world` link** with a fixed joint. Nothing supports a trigger-only
  body, so a movable root falls through the floor forever. The simulator logs a warning at
  spawn when an entity has only sensor-only colliders and a movable root.
- Triggers must be convex. Primitive shapes and the meshes the URDF Importer produces already
  are; a non-convex mesh collider is switched to convex (a convex hull of at most 255 faces)
  with a warning.
- A ray stops at the first hit, so a sensor-only shape is opaque per beam. To let some beams
  pass, make the plants thin and sparse rather than one solid block.
- Intensity does not mark the points: the lidar reports a range-only intensity. Label
  "weed" points on the consumer side from the known layout.
- Everything else about the entity is unchanged: it is listed by `get_entities`, removed by
  `delete_entity` and `reset_simulation` with `SCOPE_SPAWNED`, and `set_entity_info` can tag it.

The log line for a sensor-only material ends with `sensor_only`:

```
[CollisionMaterial] Applied 'weed' to 'weeds_link' (2 collider(s), static=0, dynamic=0, combine=Average, sensor_only)
```

## Watch the combine mode

**With the default `average`, the value you wrote is not the value you get.** Unity uses the
combine mode of whichever of the two touching materials has the **higher enum value**
(`average` < `minimum` < `multiply` < `maximum`).

When the floor has no material assigned, Unity's built-in default is the other surface
(0.6 static and dynamic, combine `average`). So

```xml
<friction static="1.0" dynamic="1.0"/>   <!-- combine omitted = average -->
```

gives an effective coefficient of **(1.0 + 0.6) / 2 = 0.8**. Use `combine="maximum"` when the
value you wrote should hold regardless of what it touches.

## When raising friction does not stop the sliding

**Past a certain speed, the robot slides no matter what the coefficient is**, because the
contact point travels further per physics step than the contact can follow. Slip measured on
diffbot (wheel radius 0.05 m) from wheel rotation versus actual travel:

| Wheel speed | Equivalent | 50 Hz (default) | 200 Hz |
|---|---|---|---|
| 3 rad/s | 0.15 m/s | 0.0 % | 1.2 % |
| 10 rad/s | 0.5 m/s | 0.0 % | 2.0 % |
| 30 rad/s | 1.5 m/s | **74.2 %** | **7.2 %** |
| 60 rad/s | 3.0 m/s | 95.8 % | 92.1 % |

The friction coefficients are identical in all four rows (`static=1.0 dynamic=1.0`); only
`Fixed Timestep` changed. Slip at 1.5 m/s falling from 74 % to 7 % shows that **at speed the
timestep dominates, not the friction setting**.

At 50 Hz the contact point moves 3 cm per step, about 34° of rotation for a 5 cm wheel. The
contact patch cannot be maintained in that regime.

Options:

- keep working speeds around 0.5 m/s or below, where slip is below the measurement floor at
  the default 50 Hz
- lower `Project Settings > Time > Fixed Timestep`, at a cost to whole-scene compute
- use a larger wheel radius, so less rotation happens per step

## Checking that the settings took

- **Unit tests**: `Packages/UrdfProperties/Tests/Runtime/CollisionMaterialApplierTests.cs` reads the
  colliders' `staticFriction` / `dynamicFriction` / `frictionCombine` / `contactOffset` /
  `isTrigger` back and checks them against the URDF. The assembly targets all platforms, so
  the test runner treats it as PlayMode:

  ```bash
  Unity -batchmode -nographics -projectPath <this repo> -runTests -testPlatform playmode \
        -assemblyNames UrdfPropertyTests -testResults results.xml -logFile unity.log
  ```
- **Scenario test**: `sim_test_utils/examples/test_sensor_only_collision.py` in the
  companion workspace spawns a sensor-only weed, drops a box through it (no contact) and
  checks that a lidar returns it at the expected range.
- **On a running simulator**: one line per application appears in the log.

  ```
  [CollisionMaterial] Applied 'wheel' to 'left_wheel_link' (1 collider(s), static=1, dynamic=1, combine=Average)
  ```

  A misspelt link name or an undefined reference is reported as a warning.
- **Behaviour**: conformance scenario C5b measures travel against wheel rotation
  ([Service-Conformance-Test.md](Service-Conformance-Test.md)).

## Colour

Colour uses the standard `<material><color rgba="..."/></material>`, which the URDF Importer
applies. That alpha survives is covered by the importer's own tests
(`Tests/Runtime/Extensions/UrdfMaterialTests.cs`).
