# collision_material — setting friction from URDF

URDF has no standard element for contact friction, so this simulator reads a custom
`<collision_material>`. The URDF Importer does not know the element, so the simulator
applies it after import
(`Assets/Scripts/UrdfProperties/CollisionMaterialApplier.cs`).

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

With several `<collision>` elements on one link, **the i-th `<collision>` maps to the i-th
shape**. If one `<collision>` expands into several colliders (submeshes, say), all of them
get the material.

The old name `<physics_material>` still parses, with a warning.

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

- **Unit tests**: `Assets/Tests/UrdfPropertyTests/CollisionMaterialApplierTests.cs` reads the
  colliders' `staticFriction` / `dynamicFriction` / `frictionCombine` / `contactOffset` back
  and checks them against the URDF.
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
