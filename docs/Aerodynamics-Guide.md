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
- [Blade Element Theory (BET)](#blade-element-theory-bet)
- [Unsteady Aerodynamic Effects](#unsteady-aerodynamic-effects)
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

  <!-- Blade Element Theory (optional - for improved accuracy) -->
  <blade_element_theory>
    <num_elements>8</num_elements>           <!-- Spanwise elements (1=original) -->
    <induced_velocity_model>prandtl</induced_velocity_model>  <!-- none/prandtl/momentum/simpleWake -->
    <taper_ratio>1.0</taper_ratio>           <!-- tip_chord/root_chord -->
    <twist_root>0</twist_root>               <!-- Root twist in degrees -->
    <twist_tip>-3</twist_tip>                <!-- Tip twist in degrees -->
    <sweep_angle>0</sweep_angle>             <!-- Sweep angle in degrees -->
  </blade_element_theory>

  <!-- Unsteady Aerodynamics (optional - for flapping/maneuvering) -->
  <unsteady_effects enable="true">
    <enable_added_mass>true</enable_added_mass>
    <added_mass_coefficient>0.75</added_mass_coefficient>  <!-- 0.75=elliptic, 1.0=flat plate -->
    <enable_wagner_lag>true</enable_wagner_lag>
    <wagner_time_constant>4.0</wagner_time_constant>
  </unsteady_effects>
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

### Blade Element Theory Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `num_elements` | 1 | 1-32 | Number of spanwise elements (1 = original quasi-steady) |
| `induced_velocity_model` | prandtl | - | Induced velocity model: `none`, `prandtl`, `momentum`, `simpleWake` |
| `taper_ratio` | 1.0 | 0.1-1.5 | Tip chord / root chord (1.0 = rectangular) |
| `twist_root` | 0 | - | Geometric twist at root in degrees |
| `twist_tip` | 0 | - | Geometric twist at tip in degrees (negative = washout) |
| `sweep_angle` | 0 | - | Sweep angle in degrees (positive = swept back) |

**Induced Velocity Models:**

| Model | Use Case | Description |
|-------|----------|-------------|
| `none` | Debugging | No induced velocity correction |
| `prandtl` | Fixed-wing | Prandtl tip-loss correction (recommended for aircraft) |
| `momentum` | Rotors | Momentum theory for propellers/rotors |
| `simpleWake` | Flapping | Simple wake model with time lag for flapping wings |

### Unsteady Aerodynamics Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `enable` (attribute) | false | - | Enable unsteady aerodynamic effects |
| `enable_added_mass` | true | - | Enable added mass (virtual mass) effect |
| `added_mass_coefficient` | 0.75 | 0.5-1.5 | Added mass coefficient (0.75 for elliptic, 1.0 for flat plate) |
| `enable_wagner_lag` | true | - | Enable Wagner function circulation lag |
| `wagner_time_constant` | 4.0 | 1-10 | Wagner time constant multiplier |

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

## Blade Element Theory (BET)

Blade Element Theory improves aerodynamic accuracy by discretizing the wing into multiple spanwise elements, each calculated independently. This enables:

- **Spanwise load distribution** - Captures how lift varies from root to tip
- **Tip-loss effects** - Accurate modeling of finite wing induced drag
- **Geometric variation** - Taper, twist, and sweep effects
- **Per-element induced velocity** - More accurate vortex/wake modeling

### When to Use BET

| Configuration | num_elements | Model |
|--------------|--------------|-------|
| Simple aircraft (fast simulation) | 1 | - |
| Fixed-wing aircraft (accurate) | 4-8 | `prandtl` |
| Propellers/rotors | 8-16 | `momentum` |
| Flapping wings | 8-16 | `simpleWake` |

### Taper and Twist

**Taper Ratio** defines how chord varies along the span:
- `taper_ratio = 1.0`: Rectangular wing (constant chord)
- `taper_ratio = 0.5`: Tip chord is half of root chord
- `taper_ratio < 1.0`: Tapered wing (common for aircraft)

**Twist Distribution** (washout):
- `twist_root = 0°, twist_tip = -3°`: Typical washout
- Washout delays tip stall, improving handling characteristics
- Linear interpolation between root and tip

### Induced Velocity Models

#### Prandtl (Fixed-Wing Aircraft)

Based on Prandtl's lifting-line theory with tip-loss correction:

```
F = (2/π) × acos(exp(-f))
f = (AR/2) × (1 - r/R) / (r/R × |sin(φ)|)
```

Best for:
- Conventional fixed-wing aircraft
- Gliders
- High aspect ratio wings

#### Momentum (Rotors/Propellers)

Based on actuator disk momentum theory:

```
v_i = √(T / (2 × ρ × A))
```

Best for:
- Helicopter rotors
- Multicopter propellers
- Ducted fans

#### SimpleWake (Flapping Wings)

Quasi-steady approximation with time-lagged wake effects:

```
v_i(t) = α × v_i_qs + (1-α) × v_i(t-Δt)
```

Best for:
- Flapping wing robots
- Bio-inspired swimmers
- Oscillating foils

---

## Unsteady Aerodynamic Effects

For rapidly changing flight conditions (flapping, maneuvering), unsteady effects become significant. Two primary effects are modeled:

### Added Mass (Virtual Mass)

When a body accelerates through fluid, it must also accelerate the surrounding fluid. This creates an "added mass" effect:

```
F_added = -m_added × a
m_added = (π/4) × ρ × c² × b × k
```

Where:
- `c` = chord length
- `b` = span length
- `k` = added mass coefficient (0.75 for elliptic, 1.0 for flat plate)
- `a` = acceleration

### Wagner Function (Circulation Lag)

When angle of attack changes suddenly, circulation (and lift) does not respond instantaneously. The Wagner function models this lag:

```
Φ(s) = 1 - 0.165×exp(-0.0455×s) - 0.335×exp(-0.3×s)
```

Where `s = 2×V×t/c` is the number of semi-chords traveled.

**Effect:**
- Sudden AoA increase → lift rises gradually (not instantly)
- Reduces lift overshoot during rapid maneuvers
- Important for flapping wing analysis

### When to Enable Unsteady Effects

| Application | Added Mass | Wagner Lag |
|-------------|------------|------------|
| Steady cruise flight | No | No |
| Aggressive maneuvering | Yes | Optional |
| Flapping wings | Yes | Yes |
| Gust response | Optional | Yes |
| Underwater oscillating fins | Yes | Yes |

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

### Example 4: Fixed-Wing with Blade Element Theory

```xml
<?xml version="1.0"?>
<robot name="accurate_aircraft">
  <aerodynamics>
    <fluid_density>1.225</fluid_density>
  </aerodynamics>

  <link name="main_wing">
    <collision>
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>6.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>
        <control_surface type="roll" multiplier="1"/>

        <!-- Blade Element Theory for accurate spanwise distribution -->
        <blade_element_theory>
          <num_elements>8</num_elements>
          <induced_velocity_model>prandtl</induced_velocity_model>
          <taper_ratio>0.6</taper_ratio>
          <twist_root>2</twist_root>
          <twist_tip>-2</twist_tip>
        </blade_element_theory>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 6.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Example 5: Flapping Wing Robot

```xml
<?xml version="1.0"?>
<robot name="flapping_wing">
  <aerodynamics>
    <fluid_medium>auto</fluid_medium>
    <air_density>1.225</air_density>
    <water_density>1027</water_density>
  </aerodynamics>

  <link name="left_wing">
    <collision>
      <aerodynamic_surface>
        <chord>0.08</chord>
        <span>0.15</span>
        <lift_slope>5.5</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <!-- Higher stall angles for dynamic stall -->
        <stall_angle_high_air>25</stall_angle_high_air>
        <stall_angle_low_air>-25</stall_angle_low_air>
        <stall_angle_high_water>20</stall_angle_high_water>
        <stall_angle_low_water>-20</stall_angle_low_water>
        <skin_friction_air>0.015</skin_friction_air>
        <skin_friction_water>0.008</skin_friction_water>

        <!-- Blade Element Theory for flapping -->
        <blade_element_theory>
          <num_elements>12</num_elements>
          <induced_velocity_model>simpleWake</induced_velocity_model>
          <taper_ratio>0.4</taper_ratio>
          <twist_root>0</twist_root>
          <twist_tip>-5</twist_tip>
        </blade_element_theory>

        <!-- Unsteady effects are critical for flapping -->
        <unsteady_effects enable="true">
          <enable_added_mass>true</enable_added_mass>
          <added_mass_coefficient>0.75</added_mass_coefficient>
          <enable_wagner_lag>true</enable_wagner_lag>
          <wagner_time_constant>4.0</wagner_time_constant>
        </unsteady_effects>
      </aerodynamic_surface>
      <geometry>
        <box size="0.08 0.005 0.15"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Example 6: Propeller with Momentum Theory

```xml
<?xml version="1.0"?>
<robot name="quadcopter">
  <aerodynamics>
    <fluid_density>1.225</fluid_density>
  </aerodynamics>

  <link name="propeller_1">
    <collision>
      <aerodynamic_surface>
        <chord>0.02</chord>
        <span>0.127</span>  <!-- 5 inch prop radius -->
        <lift_slope>5.7</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high>12</stall_angle_high>
        <stall_angle_low>-12</stall_angle_low>
        <skin_friction>0.02</skin_friction>

        <!-- Blade Element Theory for rotor -->
        <blade_element_theory>
          <num_elements>16</num_elements>
          <induced_velocity_model>momentum</induced_velocity_model>
          <taper_ratio>0.7</taper_ratio>
          <twist_root>25</twist_root>
          <twist_tip>5</twist_tip>
        </blade_element_theory>
      </aerodynamic_surface>
      <geometry>
        <cylinder radius="0.127" length="0.005"/>
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

7. **Blade Element Theory:**
   - Start with `num_elements=1` for simple testing, increase for accuracy
   - Use 4-8 elements for fixed-wing aircraft (good balance of accuracy vs. performance)
   - Use 8-16 elements for rotors/propellers or flapping wings
   - Choose appropriate induced velocity model:
     - `prandtl` for conventional aircraft (tip-loss correction)
     - `momentum` for rotors/propellers (actuator disk theory)
     - `simpleWake` for flapping/oscillating motions
   - Typical taper ratios: 0.4-0.8 for efficient wings
   - Typical washout (twist): -2° to -5° tip relative to root

8. **Unsteady Effects:**
   - Only enable for rapidly changing conditions (flapping, aggressive maneuvers)
   - Added mass is most important at low speeds and high accelerations
   - Wagner lag is important when AoA changes rapidly
   - Higher `wagner_time_constant` = slower lift response (more lag)
   - Keep `added_mass_coefficient` at 0.75 for most airfoils

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

### BET results differ from single-element
- This is expected - BET captures spanwise effects that single-element misses
- For rectangular wings with no taper/twist, results should be within ~5%
- Check induced velocity model is appropriate for your application

### Flapping wing lift seems delayed
- Wagner lag is working correctly - lift builds up over several chord lengths
- If too much lag, reduce `wagner_time_constant`
- Ensure time step is small enough for high-frequency flapping

### Propeller/rotor thrust too low
- Check induced velocity model is set to `momentum`
- Verify twist distribution is correct (high at root, low at tip)
- Ensure enough elements to capture radial variation (8-16 recommended)

---

## References

- Khan, W. & Nahon, M. (2015). Real-time modeling of agile fixed-wing UAV aerodynamics
- Anderson, J.D. (2016). Fundamentals of Aerodynamics
- Aircraft-Physics Unity Package by JoeBrow
- Leishman, J.G. (2006). Principles of Helicopter Aerodynamics (Blade Element Theory)
- Wagner, H. (1925). Über die Entstehung des dynamischen Auftriebes von Tragflügeln
- Jones, R.T. (1938). Operational treatment of the non-uniform lift theory (Wagner function approximation)
- Theodorsen, T. (1935). General theory of aerodynamic instability and the mechanism of flutter
