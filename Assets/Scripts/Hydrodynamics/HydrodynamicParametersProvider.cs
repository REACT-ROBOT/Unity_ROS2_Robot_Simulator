// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Global provider for hydrodynamic parameters
// Standalone implementation (no MARUS dependency)

using UnityEngine;

namespace Hydrodynamics
{
    /// <summary>
    /// Singleton provider that supplies hydrodynamic parameters globally.
    /// Can be used to share parameters between multiple HydrodynamicFloatingObject instances.
    /// </summary>
    public class HydrodynamicParametersProvider : MonoBehaviour
    {
        public static HydrodynamicParametersProvider current { get; private set; }

        [Header("Configuration Source")]
        [SerializeField]
        [Tooltip("Use ScriptableObject asset for configuration")]
        private bool useAsset = false;

        [SerializeField]
        private HydrodynamicConfig configAsset;

        [SerializeField]
        private HydrodynamicParameters inlineParameters = new HydrodynamicParameters();

        // Property accessors for easy access
        public float velocityReference => Parameters.velocityReference;
        public float C_PD1 => Parameters.C_PD1;
        public float C_PD2 => Parameters.C_PD2;
        public float f_P => Parameters.f_P;
        public float C_SD1 => Parameters.C_SD1;
        public float C_SD2 => Parameters.C_SD2;
        public float f_S => Parameters.f_S;
        public float slammingPower => Parameters.slammingPower;
        public float maxAcceleration => Parameters.maxAcceleration;
        public float slammingMultiplier => Parameters.slammingMultiplier;

        public HydrodynamicParameters Parameters
        {
            get
            {
                if (useAsset && configAsset != null)
                    return configAsset.parameters;
                return inlineParameters;
            }
        }

        private void Awake()
        {
            if (current != null && current != this)
            {
                Debug.LogWarning($"[HydrodynamicParametersProvider] Multiple instances detected. Using: {current.name}");
                Destroy(this);
                return;
            }
            current = this;
        }

        /// <summary>
        /// Updates the inline parameters at runtime
        /// </summary>
        public void SetParameters(HydrodynamicParameters newParameters)
        {
            inlineParameters = newParameters;
        }

        /// <summary>
        /// Creates default provider if none exists
        /// </summary>
        public static HydrodynamicParametersProvider GetOrCreate()
        {
            if (current != null)
                return current;

            var existing = FindFirstObjectByType<HydrodynamicParametersProvider>();
            if (existing != null)
            {
                current = existing;
                return current;
            }

            var go = new GameObject("HydrodynamicParametersProvider");
            current = go.AddComponent<HydrodynamicParametersProvider>();
            return current;
        }
    }
}
