// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Hydrodynamic parameters for MARUS physics integration
// Based on MARUS DebugPhysics.cs by LABUST

using System;
using UnityEngine;

namespace Hydrodynamics
{
    /// <summary>
    /// Serializable hydrodynamic parameters for underwater physics simulation.
    /// These parameters control pressure drag, suction drag, and slamming forces.
    /// Can be configured via Inspector or loaded from URDF.
    /// </summary>
    [Serializable]
    public class HydrodynamicParameters
    {
        [Header("Fluid Properties")]
        [Tooltip("Water density in kg/m^3 (default: 1027 for ocean water)")]
        public float waterDensity = 1027f;

        [Tooltip("Air density in kg/m^3")]
        public float airDensity = 1.225f;

        [Tooltip("Kinematic viscosity of water in m^2/s (default: 1e-6 at 20°C)")]
        public float waterViscosity = 0.000001f;

        [Header("Pressure Drag")]
        [Tooltip("Reference velocity for pressure drag calculations (m/s)")]
        public float velocityReference = 1f;

        [Tooltip("Pressure drag coefficient 1 (linear term)")]
        public float C_PD1 = 10f;

        [Tooltip("Pressure drag coefficient 2 (quadratic term)")]
        public float C_PD2 = 10f;

        [Tooltip("Pressure drag falloff power (should be < 1)")]
        [Range(0.1f, 1f)]
        public float f_P = 0.5f;

        [Header("Suction Drag")]
        [Tooltip("Suction drag coefficient 1 (linear term)")]
        public float C_SD1 = 10f;

        [Tooltip("Suction drag coefficient 2 (quadratic term)")]
        public float C_SD2 = 10f;

        [Tooltip("Suction drag falloff power (should be < 1)")]
        [Range(0.1f, 1f)]
        public float f_S = 0.5f;

        [Header("Slamming Force")]
        [Tooltip("Slamming force ramp-up power (should be >= 2)")]
        public float slammingPower = 2f;

        [Tooltip("Maximum acceleration for slamming normalization")]
        public float maxAcceleration = 10f;

        [Tooltip("Slamming force multiplier")]
        public float slammingMultiplier = 1f;

        [Header("Air Resistance")]
        [Tooltip("Air resistance coefficient")]
        public float airResistanceCoefficient = 0.1f;

        [Header("Viscous Water Resistance")]
        [Tooltip("Enable viscous water resistance calculation")]
        public bool enableViscousResistance = true;

        [Tooltip("Enable pressure drag calculation")]
        public bool enablePressureDrag = true;

        [Tooltip("Enable slamming force calculation")]
        public bool enableSlammingForce = true;

        [Tooltip("Enable air resistance calculation")]
        public bool enableAirResistance = true;

        /// <summary>
        /// Creates default parameters suitable for most underwater robots
        /// </summary>
        public static HydrodynamicParameters CreateDefault()
        {
            return new HydrodynamicParameters();
        }

        /// <summary>
        /// Creates parameters optimized for high-speed surface vessels (planing hulls)
        /// </summary>
        public static HydrodynamicParameters CreateForSurfaceVessel()
        {
            return new HydrodynamicParameters
            {
                velocityReference = 5f,
                C_PD1 = 15f,
                C_PD2 = 20f,
                f_P = 0.3f,
                C_SD1 = 15f,
                C_SD2 = 20f,
                f_S = 0.3f,
                slammingPower = 3f,
                maxAcceleration = 20f,
                slammingMultiplier = 1.5f
            };
        }

        /// <summary>
        /// Creates parameters optimized for slow-moving underwater vehicles (ROV/AUV)
        /// </summary>
        public static HydrodynamicParameters CreateForUnderwaterVehicle()
        {
            return new HydrodynamicParameters
            {
                velocityReference = 2f,
                C_PD1 = 5f,
                C_PD2 = 5f,
                f_P = 0.7f,
                C_SD1 = 5f,
                C_SD2 = 5f,
                f_S = 0.7f,
                slammingPower = 2f,
                maxAcceleration = 5f,
                slammingMultiplier = 0.5f,
                enableAirResistance = false
            };
        }
    }

    /// <summary>
    /// ScriptableObject wrapper for HydrodynamicParameters to allow asset-based configuration
    /// </summary>
    [CreateAssetMenu(fileName = "HydrodynamicConfig", menuName = "Hydrodynamics/Configuration", order = 1)]
    public class HydrodynamicConfig : ScriptableObject
    {
        public HydrodynamicParameters parameters = new HydrodynamicParameters();
    }
}
