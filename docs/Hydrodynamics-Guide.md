# Hydrodynamics Modeling Guide

This guide explains how to configure hydrodynamic properties for underwater and surface robot models in Unity ROS2 Robot Simulator.

## Overview

The simulator supports two levels of water physics:
1. **Basic Buoyancy** - Simple buoyancy using `buoyancy_material` (NaughtyWaterBuoyancy)
2. **Full Hydrodynamics** - Advanced physics including viscous resistance, pressure drag, slamming forces, and air resistance (MARUS-based)

## Table of Contents

- [Buoyancy Material Configuration](#buoyancy-material-configuration)
- [Hydrodynamics Parameters](#hydrodynamics-parameters)
- [URDF Configuration Examples](#urdf-configuration-examples)
- [Parameter Reference](#parameter-reference)
- [Parameter Derivation Guide](#parameter-derivation-guide)

---

## Buoyancy Material Configuration

### Basic Concept

The `buoyancy_material` tag defines **relative density (specific gravity)**, which is the ratio of material density to water density. This determines buoyancy behavior:
- `density < 1.0` → Object floats (lighter than water)
- `density = 1.0` → Neutral buoyancy (same as water)
- `density > 1.0` → Object sinks (heavier than water)

### URDF Syntax

Define materials at the robot level, then reference them in collision elements:

```xml
<robot name="my_underwater_robot">
  <!-- Define buoyancy materials (relative density / specific gravity) -->
  <buoyancy_material name="foam">
    <density value="0.05"/>  <!-- 5% of water density - strong buoyancy -->
  </buoyancy_material>

  <buoyancy_material name="aluminum">
    <density value="2.7"/>   <!-- 2.7x water density - sinks -->
  </buoyancy_material>

  <buoyancy_material name="plastic_hull">
    <density value="0.8"/>   <!-- 80% of water density - floats -->
  </buoyancy_material>

  <link name="body">
    <collision>
      <!-- Reference the material -->
      <buoyancy_material name="plastic_hull"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Common Material Relative Densities (Specific Gravity)

| Material | Relative Density | Behavior in Water |
|----------|------------------|-------------------|
| Foam (EPS) | 0.015-0.03 | Strong buoyancy |
| Syntactic foam | 0.4-0.6 | Moderate buoyancy |
| Plastic (HDPE) | 0.94-0.97 | Slight buoyancy |
| Water | 1.0 | Neutral |
| Aluminum | 2.7 | Sinks |
| Steel | 7.85 | Sinks |

**Note:** The relative density is calculated as: `material_density / water_density`. For example, aluminum with absolute density 2700 kg/m³ in water (1000 kg/m³) has relative density = 2.7.

---

## Hydrodynamics Parameters

### Enabling Hydrodynamics

Add a `<hydrodynamics>` tag to your URDF to enable advanced hydrodynamic modeling:

```xml
<robot name="my_auv">
  <!-- Robot-level defaults (applied to all links) -->
  <hydrodynamics>
    <water_density>1027</water_density>
    <!-- Additional parameters... -->
  </hydrodynamics>

  <link name="body">
    <collision>
      <!-- Link-specific overrides -->
      <hydrodynamics>
        <pressure_drag>
          <C_PD1>15</C_PD1>
        </pressure_drag>
      </hydrodynamics>
    </collision>
  </link>
</robot>
```

### Full Parameter Structure

```xml
<hydrodynamics>
  <!-- Fluid Properties -->
  <water_density>1027</water_density>     <!-- kg/m³ -->
  <air_density>1.225</air_density>        <!-- kg/m³ -->
  <velocity_reference>1.0</velocity_reference>  <!-- m/s -->

  <!-- Pressure Drag -->
  <pressure_drag>
    <C_PD1>10</C_PD1>    <!-- Linear coefficient -->
    <C_PD2>10</C_PD2>    <!-- Quadratic coefficient -->
    <f_P>0.5</f_P>       <!-- Falloff power (0.1-1.0) -->
  </pressure_drag>

  <!-- Suction Drag -->
  <suction_drag>
    <C_SD1>10</C_SD1>    <!-- Linear coefficient -->
    <C_SD2>10</C_SD2>    <!-- Quadratic coefficient -->
    <f_S>0.5</f_S>       <!-- Falloff power (0.1-1.0) -->
  </suction_drag>

  <!-- Slamming Force -->
  <slamming>
    <power>2</power>              <!-- Ramp-up power (≥2) -->
    <max_acceleration>10</max_acceleration>  <!-- m/s² -->
    <multiplier>1.0</multiplier>  <!-- Force scaling -->
  </slamming>

  <!-- Air Resistance -->
  <air_resistance>
    <coefficient>0.1</coefficient>
  </air_resistance>
</hydrodynamics>
```

---

## URDF Configuration Examples

### Example 1: Simple ROV

```xml
<?xml version="1.0"?>
<robot name="simple_rov">
  <!-- Buoyancy material for flotation foam -->
  <buoyancy_material name="flotation_foam">
    <density value="0.2"/>  <!-- 20% of water density - floats -->
  </buoyancy_material>

  <buoyancy_material name="rov_body">
    <density value="1.1"/>  <!-- 110% of water density - slightly negative buoyancy -->
  </buoyancy_material>

  <!-- Default hydrodynamics for slow-moving ROV -->
  <hydrodynamics>
    <water_density>1027</water_density>
    <velocity_reference>2.0</velocity_reference>
    <pressure_drag>
      <C_PD1>5</C_PD1>
      <C_PD2>5</C_PD2>
      <f_P>0.7</f_P>
    </pressure_drag>
    <suction_drag>
      <C_SD1>5</C_SD1>
      <C_SD2>5</C_SD2>
      <f_S>0.7</f_S>
    </suction_drag>
    <slamming>
      <power>2</power>
      <max_acceleration>5</max_acceleration>
      <multiplier>0.5</multiplier>
    </slamming>
  </hydrodynamics>

  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <buoyancy_material name="rov_body"/>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="flotation">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <buoyancy_material name="flotation_foam"/>
      <geometry>
        <box size="0.4 0.3 0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### Example 2: Surface Vessel with Per-Link Hydrodynamics

```xml
<?xml version="1.0"?>
<robot name="surface_vessel">
  <buoyancy_material name="hull_material">
    <density value="0.6"/>  <!-- 60% of water density - floats -->
  </buoyancy_material>

  <!-- Default hydrodynamics -->
  <hydrodynamics>
    <water_density>1027</water_density>
    <velocity_reference>5.0</velocity_reference>
  </hydrodynamics>

  <link name="hull">
    <collision>
      <buoyancy_material name="hull_material"/>
      <!-- Hull-specific: higher pressure drag for planing -->
      <hydrodynamics>
        <pressure_drag>
          <C_PD1>15</C_PD1>
          <C_PD2>25</C_PD2>
          <f_P>0.3</f_P>
        </pressure_drag>
        <slamming>
          <power>3</power>
          <max_acceleration>20</max_acceleration>
          <multiplier>1.5</multiplier>
        </slamming>
      </hydrodynamics>
      <geometry>
        <mesh filename="package://my_robot/meshes/hull.dae"/>
      </geometry>
    </collision>
  </link>

  <link name="keel">
    <collision>
      <!-- Keel-specific: streamlined, lower drag -->
      <hydrodynamics>
        <pressure_drag>
          <C_PD1>3</C_PD1>
          <C_PD2>5</C_PD2>
          <f_P>0.8</f_P>
        </pressure_drag>
      </hydrodynamics>
      <geometry>
        <box size="0.02 0.5 0.3"/>
      </geometry>
    </collision>
  </link>
</robot>
```

---

## Parameter Reference

### Force Summary

| Force Type | Formula | Description |
|------------|---------|-------------|
| Buoyancy | F = ρ × g × V | Voxel-based, wave surface tracking |
| Viscous Water Resistance | F = 0.5 × ρ × v² × S × Cf | Reynolds number based friction drag |
| Pressure Drag | F = -(C₁v + C₂v²) × A × cos^f(θ) × n | Flow angle dependent |
| Slamming Force | F = clamp(a/a_max)^p × cos(θ) × F_stop | Water entry impact |
| Air Resistance | F = 0.5 × ρ_air × v² × A × Cd | Above water only |

### Fluid Properties

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `water_density` | 1027 | kg/m³ | Seawater density (use 1000 for freshwater) |
| `air_density` | 1.225 | kg/m³ | Air density at sea level, 15°C |
| `velocity_reference` | 1.0 | m/s | Reference velocity for normalized drag calculations |

### Pressure Drag Parameters

Pressure drag opposes motion when a surface faces the flow direction.

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `C_PD1` | 10 | 0-100 | Linear pressure drag coefficient |
| `C_PD2` | 10 | 0-100 | Quadratic pressure drag coefficient |
| `f_P` | 0.5 | 0.1-1.0 | Falloff power (lower = sharper angular dependency) |

**Formula:**
```
F_pressure = -(C_PD1 × v_norm + C_PD2 × v_norm²) × A × cos(θ)^f_P × n
```
where:
- `v_norm = |velocity| / velocity_reference`
- `A` = surface area
- `θ` = angle between velocity and surface normal
- `n` = surface normal vector

### Suction Drag Parameters

Suction drag occurs on the rear-facing surfaces (wake side).

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `C_SD1` | 10 | 0-100 | Linear suction drag coefficient |
| `C_SD2` | 10 | 0-100 | Quadratic suction drag coefficient |
| `f_S` | 0.5 | 0.1-1.0 | Falloff power |

### Slamming Force Parameters

Slamming force simulates water entry impact (wave-piercing, splash impact).

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `power` | 2 | ≥2 | Ramp-up power (higher = more sudden impact) |
| `max_acceleration` | 10 | >0 | Maximum expected acceleration (m/s²) |
| `multiplier` | 1.0 | 0-5 | Force scaling factor |

**Formula:**
```
F_slamming = clamp(a / a_max)^p × cos(θ) × F_stop × multiplier
```
where:
- `a` = current acceleration magnitude
- `a_max` = max_acceleration parameter
- `p` = power parameter
- `θ` = angle between velocity and surface normal
- `F_stop` = stopping force estimate based on momentum

### Air Resistance

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `coefficient` | 0.1 | 0-2 | Air drag coefficient (Cd) |

**Formula:**
```
F_air = 0.5 × ρ_air × v² × A × Cd
```
(Applied opposite to velocity direction)

---

## Parameter Derivation Guide

### Estimating Pressure Drag Coefficients

1. **From CFD Analysis:**
   - Run CFD simulation at reference velocity
   - Extract pressure distribution on surfaces
   - `C_PD2 ≈ 2 × (measured drag force) / (ρ × v² × A)`

2. **From Empirical Data:**
   - Flat plate: `C_PD2 ≈ 1.0-1.3`
   - Streamlined body: `C_PD2 ≈ 0.04-0.1`
   - Bluff body (box): `C_PD2 ≈ 1.0-2.1`

3. **Rule of Thumb:**
   ```
   C_PD1 = C_PD2 / velocity_reference  (for linear behavior at low speed)
   ```

### Choosing Falloff Power (f_P, f_S)

- **f = 0.3-0.5:** Sharp angular dependency, good for flat surfaces
- **f = 0.5-0.7:** Moderate, suitable for most curved hulls
- **f = 0.7-1.0:** Gradual falloff, good for streamlined shapes

### Slamming Force Tuning

1. **power parameter:**
   - `power = 2`: Gradual ramp-up, for slow water entry
   - `power = 3-4`: Moderate, typical boats
   - `power = 5+`: Sharp impact, high-speed planing hulls

2. **max_acceleration:**
   - ROV/AUV: 5-10 m/s²
   - Surface vessel: 10-20 m/s²
   - High-speed boat: 20-50 m/s²

3. **multiplier tuning:**
   - Start with 1.0
   - Increase if impacts feel too soft
   - Decrease if simulation becomes unstable

### Viscous Resistance (Built-in)

The simulator automatically calculates viscous resistance using the ITTC 1957 friction formula:

**Force Formula:**
```
F_viscous = 0.5 × ρ × v² × S × Cf
```

**Friction Coefficient (ITTC 1957):**
```
Cf = 0.075 / (log₁₀(Rn) - 2)²
Rn = v × L / ν
```

where:
- `ρ` = water density
- `v` = flow velocity
- `S` = wetted surface area
- `Rn` = Reynolds number
- `L` = characteristic length
- `ν` = kinematic viscosity (1×10⁻⁶ m²/s for water at 20°C)

---

## Best Practices

1. **Start Simple:** Begin with default parameters and adjust based on observed behavior.

2. **Use Per-Link Parameters for Complex Robots:**
   - Define robot-level defaults
   - Override only the parameters that differ for specific links

3. **Validate Against Real Data:**
   - If possible, compare simulated drag with experimental or CFD data
   - Adjust coefficients to match measured forces

4. **Consider Operating Conditions:**
   - Freshwater vs seawater density
   - Temperature affects viscosity
   - Depth affects buoyancy slightly (water compressibility)

5. **Debug Visualization:**
   - Enable `showDebugForces` in the `HydrodynamicFloatingObject` Inspector
   - Visualize voxels with `showDebugVoxels`

---

## Troubleshooting

### Robot sinks too fast
- Increase buoyancy material volume or decrease density
- Check that `buoyancy_material` is correctly referenced

### Excessive oscillation at water surface
- Increase damping (adjust `linearDampingInWater` in Inspector)
- Reduce `slammingMultiplier`
- Check voxel resolution (`normalizedVoxelSize`)

### Unrealistic drag forces
- Verify `velocity_reference` matches expected operating speed
- Adjust `C_PD1`/`C_PD2` based on shape
- Check surface area calculation (mesh vs. box approximation)

### Simulation instability
- Reduce `slammingMultiplier` and `slammingPower`
- Decrease pressure drag coefficients
- Increase physics timestep resolution

---

## References

- MARUS (Marine Robotics Unity Simulator) - LABUST
- ITTC 1957 Friction Line - International Towing Tank Conference
- Fossen, T.I. (2011). Handbook of Marine Craft Hydrodynamics and Motion Control
- NaughtyWaterBuoyancy - Unity Asset Store
