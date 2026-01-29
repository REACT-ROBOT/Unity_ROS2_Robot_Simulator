// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Aerodynamic surface parameters for lift, drag, and moment calculations
// Based on Aircraft-Physics by JoeBrow (MIT License) and Khan & Nahon 2015

using System;
using UnityEngine;

namespace Aerodynamics
{
    /// <summary>
    /// Aerodynamic medium type for automatic detection
    /// </summary>
    public enum FluidMedium
    {
        Air,
        Water,
        Auto  // Automatically detect based on water surface
    }

    /// <summary>
    /// Current detected medium state
    /// </summary>
    public enum CurrentMedium
    {
        Air,
        Water,
        Transition  // Partially submerged
    }

    /// <summary>
    /// Induced velocity model type for blade element theory
    /// </summary>
    public enum InducedVelocityModelType
    {
        /// <summary>No induced velocity correction (for comparison/debugging)</summary>
        None,
        /// <summary>Prandtl tip-loss correction (best for fixed-wing aircraft)</summary>
        Prandtl,
        /// <summary>Momentum theory (best for rotors/propellers)</summary>
        Momentum,
        /// <summary>Simple wake model (for flapping wings)</summary>
        SimpleWake
    }

    /// <summary>
    /// Serializable parameters for an aerodynamic surface.
    /// These parameters define the lift, drag, and moment characteristics.
    /// Supports both air and water environments with automatic medium detection.
    /// Can be configured via Inspector or loaded from URDF.
    /// </summary>
    [Serializable]
    public class AeroSurfaceParameters
    {
        [Header("Surface Geometry")]
        [Tooltip("Chord length in meters (wing width in flow direction)")]
        public float chord = 1f;

        [Tooltip("Span length in meters (wing length perpendicular to flow)")]
        public float span = 1f;

        [Tooltip("Auto-calculate aspect ratio from chord and span")]
        public bool autoAspectRatio = true;

        [Tooltip("Aspect ratio (span/chord). Used if autoAspectRatio is false")]
        public float aspectRatio = 1f;

        [Header("Lift Characteristics")]
        [Tooltip("Lift slope (dCl/dα) in 1/rad. Thin airfoil theory: 2π ≈ 6.28")]
        public float liftSlope = 6.28f;

        [Tooltip("Zero-lift angle of attack in degrees")]
        public float zeroLiftAoA = 0f;

        [Tooltip("Positive stall angle in degrees (air)")]
        public float stallAngleHighAir = 15f;

        [Tooltip("Negative stall angle in degrees (air)")]
        public float stallAngleLowAir = -15f;

        [Tooltip("Positive stall angle in degrees (water) - typically lower due to higher Re")]
        public float stallAngleHighWater = 12f;

        [Tooltip("Negative stall angle in degrees (water)")]
        public float stallAngleLowWater = -12f;

        [Header("Drag Characteristics")]
        [Tooltip("Skin friction coefficient in air")]
        public float skinFrictionAir = 0.02f;

        [Tooltip("Skin friction coefficient in water (typically lower)")]
        public float skinFrictionWater = 0.01f;

        [Header("Control Surface")]
        [Tooltip("Fraction of chord that is the control surface (0-0.4 typical)")]
        [Range(0f, 0.5f)]
        public float flapFraction = 0f;

        [Tooltip("Maximum flap deflection angle in degrees")]
        public float maxFlapAngle = 50f;

        [Header("Fluid Properties")]
        [Tooltip("Fluid medium mode (Air, Water, or Auto for automatic detection)")]
        public FluidMedium fluidMedium = FluidMedium.Auto;

        [Tooltip("Air density in kg/m³ (standard: 1.225 at sea level)")]
        public float airDensity = 1.225f;

        [Tooltip("Water density in kg/m³ (freshwater: 1000, seawater: 1027)")]
        public float waterDensity = 1027f;

        [Tooltip("Kinematic viscosity of air in m²/s (standard: 1.5e-5 at 20°C)")]
        public float airKinematicViscosity = 1.5e-5f;

        [Tooltip("Kinematic viscosity of water in m²/s (standard: 1e-6 at 20°C)")]
        public float waterKinematicViscosity = 1e-6f;

        [Header("Cavitation (Water Only)")]
        [Tooltip("Enable cavitation effects at high speeds in water")]
        public bool enableCavitation = false;

        [Tooltip("Cavitation number threshold (typically 0.2-1.0)")]
        public float cavitationThreshold = 0.5f;

        [Header("Blade Element Theory")]
        [Tooltip("Number of spanwise elements (1 = original quasi-steady model)")]
        [Range(1, 32)]
        public int numElements = 1;

        [Tooltip("Induced velocity model for finite wing effects")]
        public InducedVelocityModelType inducedVelocityModel = InducedVelocityModelType.Prandtl;

        [Tooltip("Maximum iterations for induced velocity convergence")]
        [Range(1, 20)]
        public int maxInducedVelocityIterations = 5;

        [Tooltip("Convergence tolerance for induced velocity (relative change)")]
        [Range(0.0001f, 0.1f)]
        public float inducedVelocityTolerance = 0.01f;

        [Tooltip("Relaxation factor for induced velocity iteration (0.5-1.0 typical)")]
        [Range(0.3f, 1.0f)]
        public float inducedVelocityRelaxation = 0.7f;

        [Tooltip("Taper ratio (tip chord / root chord). 1.0 = rectangular wing")]
        [Range(0.1f, 1.5f)]
        public float taperRatio = 1.0f;

        [Tooltip("Geometric twist at wing root in degrees (positive = nose up)")]
        public float twistRoot = 0f;

        [Tooltip("Geometric twist at wing tip in degrees")]
        public float twistTip = 0f;

        [Tooltip("Sweep angle in degrees (positive = swept back)")]
        public float sweepAngle = 0f;

        [Header("Unsteady Aerodynamics")]
        [Tooltip("Enable unsteady aerodynamic effects (for flapping/maneuvering)")]
        public bool enableUnsteadyEffects = false;

        [Tooltip("Enable added mass (virtual mass) effect")]
        public bool enableAddedMass = true;

        [Tooltip("Added mass coefficient (0.75 for elliptic, 1.0 for flat plate)")]
        [Range(0.5f, 1.5f)]
        public float addedMassCoefficient = 0.75f;

        [Tooltip("Enable Wagner function circulation lag")]
        public bool enableWagnerLag = true;

        [Tooltip("Wagner time constant multiplier (typical: 4.0)")]
        [Range(1f, 10f)]
        public float wagnerTimeConstant = 4.0f;

        /// <summary>
        /// Gets the effective aspect ratio
        /// </summary>
        public float EffectiveAspectRatio => autoAspectRatio ? span / chord : aspectRatio;

        /// <summary>
        /// Gets the surface area in m²
        /// </summary>
        public float Area => chord * span;

        /// <summary>
        /// Gets the characteristic length (chord) for Reynolds number calculation
        /// </summary>
        public float CharacteristicLength => chord;

        /// <summary>
        /// Gets the fluid density for the specified medium
        /// </summary>
        public float GetDensity(CurrentMedium medium)
        {
            return medium == CurrentMedium.Water ? waterDensity : airDensity;
        }

        /// <summary>
        /// Gets the kinematic viscosity for the specified medium
        /// </summary>
        public float GetKinematicViscosity(CurrentMedium medium)
        {
            return medium == CurrentMedium.Water ? waterKinematicViscosity : airKinematicViscosity;
        }

        /// <summary>
        /// Gets the skin friction for the specified medium
        /// </summary>
        public float GetSkinFriction(CurrentMedium medium)
        {
            return medium == CurrentMedium.Water ? skinFrictionWater : skinFrictionAir;
        }

        /// <summary>
        /// Gets the stall angles for the specified medium
        /// </summary>
        public (float high, float low) GetStallAngles(CurrentMedium medium)
        {
            if (medium == CurrentMedium.Water)
                return (stallAngleHighWater, stallAngleLowWater);
            return (stallAngleHighAir, stallAngleLowAir);
        }

        /// <summary>
        /// Calculates Reynolds number for given velocity and medium
        /// Re = V × L / ν
        /// </summary>
        public float CalculateReynoldsNumber(float velocity, CurrentMedium medium)
        {
            float viscosity = GetKinematicViscosity(medium);
            if (viscosity <= 0) return 0;
            return Mathf.Abs(velocity) * CharacteristicLength / viscosity;
        }

        /// <summary>
        /// Legacy property for backward compatibility
        /// </summary>
        [Obsolete("Use stallAngleHighAir instead")]
        public float stallAngleHigh
        {
            get => stallAngleHighAir;
            set => stallAngleHighAir = value;
        }

        /// <summary>
        /// Legacy property for backward compatibility
        /// </summary>
        [Obsolete("Use stallAngleLowAir instead")]
        public float stallAngleLow
        {
            get => stallAngleLowAir;
            set => stallAngleLowAir = value;
        }

        /// <summary>
        /// Legacy property for backward compatibility
        /// </summary>
        [Obsolete("Use skinFrictionAir instead")]
        public float skinFriction
        {
            get => skinFrictionAir;
            set => skinFrictionAir = value;
        }

        /// <summary>
        /// Legacy property for backward compatibility
        /// </summary>
        [Obsolete("Use airDensity or waterDensity instead")]
        public float fluidDensity
        {
            get => fluidMedium == FluidMedium.Water ? waterDensity : airDensity;
            set
            {
                if (fluidMedium == FluidMedium.Water)
                    waterDensity = value;
                else
                    airDensity = value;
            }
        }

        /// <summary>
        /// Creates default parameters for a typical wing section (air)
        /// </summary>
        public static AeroSurfaceParameters CreateDefaultWing()
        {
            return new AeroSurfaceParameters
            {
                chord = 1.5f,
                span = 2f,
                liftSlope = 6.28f,
                zeroLiftAoA = -2f,
                stallAngleHighAir = 15f,
                stallAngleLowAir = -15f,
                stallAngleHighWater = 12f,
                stallAngleLowWater = -12f,
                skinFrictionAir = 0.02f,
                skinFrictionWater = 0.01f,
                flapFraction = 0.2f,
                fluidMedium = FluidMedium.Air
            };
        }

        /// <summary>
        /// Creates parameters for a horizontal stabilizer
        /// </summary>
        public static AeroSurfaceParameters CreateHorizontalStabilizer()
        {
            return new AeroSurfaceParameters
            {
                chord = 0.8f,
                span = 1.2f,
                liftSlope = 6.28f,
                zeroLiftAoA = 0f,
                stallAngleHighAir = 15f,
                stallAngleLowAir = -15f,
                stallAngleHighWater = 12f,
                stallAngleLowWater = -12f,
                skinFrictionAir = 0.02f,
                skinFrictionWater = 0.01f,
                flapFraction = 0.4f,  // Elevator
                fluidMedium = FluidMedium.Air
            };
        }

        /// <summary>
        /// Creates parameters for a vertical stabilizer
        /// </summary>
        public static AeroSurfaceParameters CreateVerticalStabilizer()
        {
            return new AeroSurfaceParameters
            {
                chord = 1f,
                span = 1f,
                liftSlope = 6.28f,
                zeroLiftAoA = 0f,
                stallAngleHighAir = 15f,
                stallAngleLowAir = -15f,
                stallAngleHighWater = 12f,
                stallAngleLowWater = -12f,
                skinFrictionAir = 0.02f,
                skinFrictionWater = 0.01f,
                flapFraction = 0.35f,  // Rudder
                fluidMedium = FluidMedium.Air
            };
        }

        /// <summary>
        /// Creates parameters for a fuselage body (drag-only, no lift)
        /// </summary>
        public static AeroSurfaceParameters CreateFuselageBody()
        {
            return new AeroSurfaceParameters
            {
                chord = 3f,
                span = 0.5f,
                autoAspectRatio = false,
                aspectRatio = 0.5f,
                liftSlope = 0f,  // No lift
                zeroLiftAoA = 0f,
                stallAngleHighAir = 0f,
                stallAngleLowAir = 0f,
                stallAngleHighWater = 0f,
                stallAngleLowWater = 0f,
                skinFrictionAir = 0.04f,
                skinFrictionWater = 0.02f,
                flapFraction = 0f,
                fluidMedium = FluidMedium.Air
            };
        }

        /// <summary>
        /// Creates parameters for underwater fin (hydrofoil)
        /// </summary>
        public static AeroSurfaceParameters CreateUnderwaterFin()
        {
            return new AeroSurfaceParameters
            {
                chord = 0.3f,
                span = 0.5f,
                liftSlope = 6.28f,
                zeroLiftAoA = 0f,
                stallAngleHighAir = 15f,
                stallAngleLowAir = -15f,
                stallAngleHighWater = 12f,
                stallAngleLowWater = -12f,
                skinFrictionAir = 0.02f,
                skinFrictionWater = 0.01f,
                flapFraction = 0.3f,
                fluidMedium = FluidMedium.Water
            };
        }

        /// <summary>
        /// Creates parameters for amphibious vehicle (auto-detect medium)
        /// </summary>
        public static AeroSurfaceParameters CreateAmphibiousSurface()
        {
            return new AeroSurfaceParameters
            {
                chord = 0.5f,
                span = 1.0f,
                liftSlope = 6.28f,
                zeroLiftAoA = 0f,
                stallAngleHighAir = 15f,
                stallAngleLowAir = -15f,
                stallAngleHighWater = 12f,
                stallAngleLowWater = -12f,
                skinFrictionAir = 0.02f,
                skinFrictionWater = 0.01f,
                flapFraction = 0.3f,
                fluidMedium = FluidMedium.Auto,  // Auto-detect
                enableCavitation = true
            };
        }

        /// <summary>
        /// Creates parameters for flapping wing (optimized for both air and water)
        /// </summary>
        public static AeroSurfaceParameters CreateFlappingWing()
        {
            return new AeroSurfaceParameters
            {
                chord = 0.1f,
                span = 0.3f,
                liftSlope = 5.5f,  // Slightly lower for unsteady effects
                zeroLiftAoA = 0f,
                stallAngleHighAir = 20f,  // Higher for dynamic stall
                stallAngleLowAir = -20f,
                stallAngleHighWater = 15f,
                stallAngleLowWater = -15f,
                skinFrictionAir = 0.015f,
                skinFrictionWater = 0.008f,
                flapFraction = 0f,  // No control surface on flapping wing
                fluidMedium = FluidMedium.Auto
            };
        }
    }

    /// <summary>
    /// ScriptableObject wrapper for AeroSurfaceParameters
    /// </summary>
    [CreateAssetMenu(fileName = "AeroSurfaceConfig", menuName = "Aerodynamics/Surface Configuration", order = 1)]
    public class AeroSurfaceConfig : ScriptableObject
    {
        public AeroSurfaceParameters parameters = new AeroSurfaceParameters();
    }
}
