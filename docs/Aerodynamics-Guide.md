# Aerodynamics Modeling Guide

This guide explains how to configure aerodynamic properties for aircraft, UAV, and flapping robot models in Unity ROS2 Robot Simulator.

## Overview

The simulator supports aerodynamic physics based on the **Khan and Nahon 2015** paper, implemented through the AeroSurface system. This includes:

1. **Lift Coefficient Curve (Cl-α)** - Linear lift in normal flight, nonlinear in stall
2. **Drag Coefficient Curve (Cd-α)** - Parasitic, induced, and stall-induced drag
3. **Stall Model** - Smooth transition between normal and stalled flight
4. **Control Surfaces** - Flaps, ailerons, elevators, rudders

The system is designed for both air and underwater environments (hydrofoils).

## Table of Contents

- [Basic Concepts](#basic-concepts)
- [URDF Configuration](#urdf-configuration)
- [Parameter Reference](#parameter-reference)
- [Force Calculations](#force-calculations)
- [Control Surface Configuration](#control-surface-configuration)
- [Examples](#examples)

---

## Basic Concepts

### Aerodynamic Forces

Each aerodynamic surface generates three types of forces:

| Force | Formula | Description |
|-------|---------|-------------|
| Lift | L = Cl × q × S | Perpendicular to airflow |
| Drag | D = Cd × q × S | Parallel to airflow |
| Moment | M = Cm × q × S × c | Pitching moment about chord |

Where:
- `q = 0.5 × ρ × V²` = Dynamic pressure
- `S = chord × span` = Surface area
- `c` = Chord length
- `ρ` = Fluid density

### Angle of Attack (AoA)

The angle between the airflow direction and the chord line of the surface.

- **Low AoA** (< stall angle): Linear lift, low drag
- **Stall** (> stall angle): Lift drops, drag increases dramatically
- **Post-stall**: Separated flow, plate-like behavior

### Aspect Ratio Correction

Finite wing spans generate wingtip vortices that reduce effective lift. The corrected lift slope is:

```
Cl_corrected = Cl_slope × AR / (AR + 2×(AR+4)/(AR+2))
```

Where `AR = span / chord` is the aspect ratio.

---

## URDF Configuration

### Basic Structure

```xml
<robot name="my_aircraft">
  <!-- Robot-level default aerodynamic parameters -->
  <aerodynamics>
    <fluid_density>1.225</fluid_density>  <!-- Air at sea level -->
  </aerodynamics>

  <link name="left_wing">
    <collision>
      <!-- Per-link aerodynamic surface -->
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>2.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>

        <!-- Control surface settings -->
        <control_surface type="roll" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 2.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Full Parameter Structure

```xml
<aerodynamic_surface>
  <!-- Geometry -->
  <chord>1.0</chord>              <!-- Chord length in meters -->
  <span>1.0</span>                <!-- Span length in meters -->
  <aspect_ratio>1.0</aspect_ratio> <!-- Manual AR (optional) -->

  <!-- Lift Characteristics -->
  <lift_slope>6.28</lift_slope>   <!-- dCl/dα in 1/rad (thin airfoil: 2π) -->
  <zero_lift_aoa>0</zero_lift_aoa> <!-- Zero-lift AoA in degrees -->

  <!-- Stall angles (legacy single-value or per-medium) -->
  <stall_angle_high>15</stall_angle_high>      <!-- Both media (legacy) -->
  <stall_angle_low>-15</stall_angle_low>       <!-- Both media (legacy) -->
  <stall_angle_high_air>15</stall_angle_high_air>    <!-- Air only -->
  <stall_angle_low_air>-15</stall_angle_low_air>     <!-- Air only -->
  <stall_angle_high_water>12</stall_angle_high_water> <!-- Water only -->
  <stall_angle_low_water>-12</stall_angle_low_water>  <!-- Water only -->

  <!-- Drag Characteristics (legacy single-value or per-medium) -->
  <skin_friction>0.02</skin_friction>          <!-- Both media (legacy) -->
  <skin_friction_air>0.02</skin_friction_air>  <!-- Air only -->
  <skin_friction_water>0.01</skin_friction_water> <!-- Water only -->

  <!-- Control Surface -->
  <flap_fraction>0.0</flap_fraction>   <!-- Flap chord fraction (0-0.5) -->
  <max_flap_angle>50</max_flap_angle>  <!-- Max deflection in degrees -->
  <control_surface type="pitch|roll|yaw|flap" multiplier="1"/>

  <!-- Fluid Properties -->
  <fluid_medium>air</fluid_medium>     <!-- "air", "water", or "auto" -->

  <!-- Density (legacy single-value or per-medium) -->
  <fluid_density>1.225</fluid_density>   <!-- Legacy: auto-detect air/water -->
  <air_density>1.225</air_density>       <!-- Air density (kg/m³) -->
  <water_density>1027</water_density>    <!-- Water density (kg/m³) -->

  <!-- Kinematic Viscosity (for Reynolds number) -->
  <air_kinematic_viscosity>1.5e-5</air_kinematic_viscosity>   <!-- m²/s -->
  <water_kinematic_viscosity>1e-6</water_kinematic_viscosity> <!-- m²/s -->

  <!-- Cavitation (water only) -->
  <enable_cavitation>false</enable_cavitation>
  <cavitation_threshold>0.5</cavitation_threshold>
</aerodynamic_surface>
```

---

## Parameter Reference

### Geometry Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `chord` | 1.0 | m | Chord length (width in flow direction) |
| `span` | 1.0 | m | Span length (perpendicular to flow) |
| `aspect_ratio` | auto | - | AR = span/chord (auto-calculated if not specified) |

### Lift Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `lift_slope` | 6.28 | 0-8 | Lift curve slope (dCl/dα) in 1/rad |
| `zero_lift_aoa` | 0 | -10 to 5 | Zero-lift angle of attack (degrees) |
| `stall_angle_high` | 15 | 10-20 | Positive stall angle - both media (legacy) |
| `stall_angle_low` | -15 | -20 to -10 | Negative stall angle - both media (legacy) |
| `stall_angle_high_air` | 15 | 10-20 | Positive stall angle in air |
| `stall_angle_low_air` | -15 | -20 to -10 | Negative stall angle in air |
| `stall_angle_high_water` | 12 | 8-15 | Positive stall angle in water |
| `stall_angle_low_water` | -12 | -15 to -8 | Negative stall angle in water |

**Typical lift slopes:**
- Thin airfoil theory: 2π ≈ 6.28
- NACA 0012: ~5.7
- Flat plate: ~3.5

**Note:** Water stall angles are typically lower due to higher Reynolds numbers.

### Drag Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `skin_friction` | 0.02 | 0.01-0.05 | Skin friction - both media (legacy) |
| `skin_friction_air` | 0.02 | 0.01-0.05 | Skin friction in air |
| `skin_friction_water` | 0.01 | 0.005-0.03 | Skin friction in water |

### Control Surface Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `flap_fraction` | 0 | 0-0.5 | Fraction of chord that is flap |
| `max_flap_angle` | 50 | 30-60 | Maximum flap deflection (degrees) |

### Fluid Properties

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fluid_medium` | air | "air", "water", or "auto" (automatic detection) |
| `fluid_density` | 1.225 | Legacy: single density value (kg/m³) |
| `air_density` | 1.225 | Air density (kg/m³) |
| `water_density` | 1027 | Water density (kg/m³) - seawater |
| `air_kinematic_viscosity` | 1.5e-5 | Air kinematic viscosity (m²/s) |
| `water_kinematic_viscosity` | 1e-6 | Water kinematic viscosity (m²/s) |

### Cavitation Parameters (Water Only)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_cavitation` | false | Enable cavitation effects at high speeds |
| `cavitation_threshold` | 0.5 | Cavitation number threshold (0.2-1.0) |

---

## Force Calculations

### Lift Coefficient (Normal Flight)

In the linear region (AoA < stall angle):

```
Cl = Cl_slope_corrected × (α - α_zero_lift)
```

### Induced Drag

From Prandtl's lifting line theory:

```
α_induced = Cl / (π × AR)
α_effective = α - α_zero_lift - α_induced
```

### Drag Coefficient

```
Cd = Cn × sin(α_eff) + Ct × cos(α_eff)
```

Where:
- `Cn` = Normal force coefficient
- `Ct` = Tangential (friction) coefficient = Cf × cos(α_eff)

### Stall Model

In the stalled regime, the normal force coefficient transitions to a flat-plate model:

```
Cn = Cf_90 × sin(α_eff) × (1/(0.56 + 0.44×|sin(α_eff)|) - 0.41×(1 - exp(-17/AR)))
```

### Flap Effects

Control surface deflection modifies:
1. **Zero-lift angle**: Shifts downward with positive deflection
2. **Maximum lift coefficient**: Modified by flap fraction
3. **Stall angles**: Adjusted based on lift coefficient change

Effectiveness reduces at large deflections (80% at small angles, 40% at ±50°).

---

## Control Surface Configuration

### Control Input Types

| Type | Description | Typical Surfaces |
|------|-------------|------------------|
| `pitch` | Nose up/down control | Elevator, horizontal stabilizer |
| `roll` | Bank left/right control | Ailerons, differential wings |
| `yaw` | Nose left/right control | Rudder, vertical stabilizer |
| `flap` | Lift augmentation | Flaps, slats |

### Input Multiplier

Use `multiplier` to create differential controls:

```xml
<!-- Left aileron: roll right when control positive -->
<control_surface type="roll" multiplier="1"/>

<!-- Right aileron: roll right when control negative -->
<control_surface type="roll" multiplier="-1"/>
```

### Programmatic Control

```csharp
// Get the AerodynamicsController
var aeroController = robot.GetComponent<AerodynamicsController>();

// Set control inputs (all values -1 to 1, except flap 0 to 1)
aeroController.SetControlInputs(
    pitch: 0.5f,   // Nose up
    roll: -0.3f,   // Bank left
    yaw: 0f,       // Neutral
    flap: 0f       // No flaps
);

// Set thrust
aeroController.ThrustPercent = 0.8f;
```

---

## Examples

### Example 1: Simple Fixed-Wing Aircraft

```xml
<?xml version="1.0"?>
<robot name="simple_aircraft">
  <!-- Default aerodynamic properties -->
  <aerodynamics>
    <fluid_density>1.225</fluid_density>
  </aerodynamics>

  <!-- Main wing sections -->
  <link name="left_wing">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>3.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>
        <control_surface type="roll" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 3.0"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wing">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>3.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>
        <control_surface type="roll" multiplier="-1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 3.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Horizontal stabilizer with elevator -->
  <link name="h_stabilizer">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>0.8</chord>
        <span>1.5</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.4</flap_fraction>
        <control_surface type="pitch" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="0.8 0.05 1.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Vertical stabilizer with rudder -->
  <link name="v_stabilizer">
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.02"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>0.8</chord>
        <span>1.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.35</flap_fraction>
        <control_surface type="yaw" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="0.8 1.0 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Fuselage (drag only) -->
  <link name="fuselage">
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="5" iyz="0" izz="5"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>3.0</chord>
        <span>0.5</span>
        <aspect_ratio>0.5</aspect_ratio>
        <lift_slope>0</lift_slope>
        <stall_angle_high>0</stall_angle_high>
        <stall_angle_low>0</stall_angle_low>
        <skin_friction>0.04</skin_friction>
      </aerodynamic_surface>
      <geometry>
        <cylinder radius="0.25" length="3.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Example 2: Underwater Glider (Hydrofoil)

```xml
<?xml version="1.0"?>
<robot name="underwater_glider">
  <!-- Underwater environment -->
  <aerodynamics>
    <fluid_medium>water</fluid_medium>
    <water_density>1027</water_density>
  </aerodynamics>

  <link name="main_wing">
    <collision>
      <aerodynamic_surface>
        <chord>0.3</chord>
        <span>1.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high_water>12</stall_angle_high_water>
        <stall_angle_low_water>-12</stall_angle_low_water>
        <skin_friction_water>0.01</skin_friction_water>
        <fluid_medium>water</fluid_medium>
      </aerodynamic_surface>
      <geometry>
        <box size="0.3 0.02 1.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Example 3: Amphibious Vehicle (Auto-Detection)

```xml
<?xml version="1.0"?>
<robot name="amphibious_vehicle">
  <!-- Default aerodynamic parameters for dual-medium operation -->
  <aerodynamics>
    <fluid_medium>auto</fluid_medium>
    <air_density>1.225</air_density>
    <water_density>1027</water_density>
  </aerodynamics>

  <link name="wing">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>0.5</chord>
        <span>1.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <!-- Medium-specific stall angles -->
        <stall_angle_high_air>15</stall_angle_high_air>
        <stall_angle_low_air>-15</stall_angle_low_air>
        <stall_angle_high_water>12</stall_angle_high_water>
        <stall_angle_low_water>-12</stall_angle_low_water>
        <!-- Medium-specific skin friction -->
        <skin_friction_air>0.02</skin_friction_air>
        <skin_friction_water>0.01</skin_friction_water>
        <!-- Auto-detect medium based on water surface -->
        <fluid_medium>auto</fluid_medium>
        <!-- Enable cavitation effects when underwater -->
        <enable_cavitation>true</enable_cavitation>
        <cavitation_threshold>0.5</cavitation_threshold>
        <control_surface type="roll" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="0.5 0.05 1.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

---

## Best Practices

1. **Start with defaults:** Use lift_slope=6.28 and adjust based on observed behavior.

2. **Match geometry to collision:** The chord and span should approximate your visual/collision geometry.

3. **Use appropriate stall angles:**
   - Conventional airfoils: 12-16°
   - Thin airfoils: 8-12°
   - Underwater fins: 10-14°

4. **Control surface sizing:**
   - Ailerons: flap_fraction 0.2-0.3
   - Elevator: flap_fraction 0.3-0.4
   - Rudder: flap_fraction 0.3-0.4

5. **For underwater use:**
   - Use `skin_friction_water` (typically lower than air due to smoother flow)
   - Use `stall_angle_*_water` (typically 80% of air values due to higher Reynolds numbers)
   - Set `water_density` to ~1027 kg/m³ for seawater

6. **For amphibious/dual-medium vehicles:**
   - Set `fluid_medium` to "auto" for automatic detection
   - Specify separate parameters for air and water with `*_air` and `*_water` suffixes
   - Consider enabling cavitation for high-speed underwater operation
   - The system automatically interpolates parameters during transition (partially submerged)

---

## Troubleshooting

### Aircraft won't generate lift
- Check that lift_slope > 0
- Verify airspeed is sufficient
- Ensure surfaces are oriented correctly (lift perpendicular to chord)

### Unstable flight
- Reduce control surface sensitivity
- Add more tail area for stability
- Check center of mass position relative to aerodynamic center

### Excessive drag
- Reduce skin_friction
- Check aspect ratio (higher AR = less induced drag)
- Verify stall angles aren't too low

### Stall behavior too abrupt
- The stall transition padding is automatic (5-15°)
- Check that stall angles are realistic for your airfoil type

---

## References

- Khan, W. & Nahon, M. (2015). Real-time modeling of agile fixed-wing UAV aerodynamics
- Anderson, J.D. (2016). Fundamentals of Aerodynamics
- Aircraft-Physics Unity Package by JoeBrow
