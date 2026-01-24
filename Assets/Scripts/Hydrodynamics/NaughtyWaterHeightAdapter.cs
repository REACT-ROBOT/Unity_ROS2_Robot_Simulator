// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Bridge adapter between MARUS WaterHeightSampler and NaughtyWaterBuoyancy WaterVolume
// This allows MARUS physics components to work with NaughtyWaterBuoyancy's water system

using UnityEngine;
using NaughtyWaterBuoyancy;
using System.Collections.Generic;

namespace Hydrodynamics
{
    /// <summary>
    /// Adapter that provides MARUS-compatible water height sampling using NaughtyWaterBuoyancy's WaterVolume.
    /// Implements a singleton pattern compatible with MARUS's WaterHeightSampler interface.
    /// </summary>
    public class NaughtyWaterHeightAdapter : MonoBehaviour
    {
        private static NaughtyWaterHeightAdapter _instance;
        public static NaughtyWaterHeightAdapter Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindFirstObjectByType<NaughtyWaterHeightAdapter>();
                    if (_instance == null)
                    {
                        // Try to find WaterVolume and create adapter automatically
                        var waterVolume = FindFirstObjectByType<WaterVolume>();
                        if (waterVolume != null)
                        {
                            _instance = waterVolume.gameObject.AddComponent<NaughtyWaterHeightAdapter>();
                        }
                        else
                        {
                            // Create a new GameObject with the adapter
                            var go = new GameObject("NaughtyWaterHeightAdapter");
                            _instance = go.AddComponent<NaughtyWaterHeightAdapter>();
                            Debug.LogWarning("[NaughtyWaterHeightAdapter] No WaterVolume found. Water height will default to 0.");
                        }
                    }
                }
                return _instance;
            }
        }

        [SerializeField]
        [Tooltip("Reference to the NaughtyWaterBuoyancy WaterVolume. Auto-detected if not set.")]
        private WaterVolume waterVolume;

        [SerializeField]
        [Tooltip("Default water level when no WaterVolume is available")]
        private float defaultWaterLevel = 0f;

        [SerializeField]
        [Tooltip("Enable debug logging")]
        private bool debugMode = false;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(this);
                return;
            }
            _instance = this;

            // Auto-detect WaterVolume if not set
            if (waterVolume == null)
            {
                waterVolume = GetComponent<WaterVolume>();
                if (waterVolume == null)
                {
                    waterVolume = FindFirstObjectByType<WaterVolume>();
                }
            }

            if (waterVolume != null && debugMode)
            {
                Debug.Log($"[NaughtyWaterHeightAdapter] Connected to WaterVolume: {waterVolume.name}");
            }
        }

        /// <summary>
        /// Sets the WaterVolume reference. Use this when water is created dynamically.
        /// </summary>
        public void SetWaterVolume(WaterVolume volume)
        {
            waterVolume = volume;
            if (debugMode)
            {
                Debug.Log($"[NaughtyWaterHeightAdapter] WaterVolume set to: {volume?.name ?? "null"}");
            }
        }

        /// <summary>
        /// Gets the current WaterVolume reference.
        /// </summary>
        public WaterVolume GetWaterVolume()
        {
            return waterVolume;
        }

        /// <summary>
        /// Returns the water level at the given world position.
        /// Compatible with MARUS WaterHeightSampler.GetWaterLevel()
        /// </summary>
        /// <param name="position">World position to query</param>
        /// <param name="minSpatialLength">Minimum spatial length (ignored, for API compatibility)</param>
        /// <returns>Water height at the position</returns>
        public float GetWaterLevel(Vector3 position, float minSpatialLength = 0.5f)
        {
            if (waterVolume != null)
            {
                return waterVolume.GetWaterLevel(position);
            }
            return defaultWaterLevel;
        }

        /// <summary>
        /// Batch query for water levels at multiple positions.
        /// Compatible with MARUS WaterHeightSampler.GetWaterLevel(Vector3[], float[], int)
        /// </summary>
        /// <param name="i_points">Array of world positions to query</param>
        /// <param name="o_heights">Output array for water heights</param>
        /// <param name="i_array_size">Number of points to process</param>
        /// <param name="i_minSpatialLength">Minimum spatial length (ignored, for API compatibility)</param>
        public void GetWaterLevel(Vector3[] i_points, float[] o_heights, int i_array_size, float i_minSpatialLength = 0.5f)
        {
            if (waterVolume != null)
            {
                for (int i = 0; i < i_array_size; i++)
                {
                    o_heights[i] = waterVolume.GetWaterLevel(i_points[i]);
                }
            }
            else
            {
                for (int i = 0; i < i_array_size; i++)
                {
                    o_heights[i] = defaultWaterLevel;
                }
            }
        }

        /// <summary>
        /// Returns water levels for a list of points.
        /// Compatible with MARUS WaterHeightSampler.GetWaterLevel(List<Vector3>)
        /// </summary>
        /// <param name="points">List of world positions to query</param>
        /// <param name="minSpatialLength">Minimum spatial length (ignored, for API compatibility)</param>
        /// <returns>Array of water heights</returns>
        public float[] GetWaterLevel(List<Vector3> points, float minSpatialLength = 0.5f)
        {
            float[] heights = new float[points.Count];

            if (waterVolume != null)
            {
                for (int i = 0; i < points.Count; i++)
                {
                    heights[i] = waterVolume.GetWaterLevel(points[i]);
                }
            }
            else
            {
                for (int i = 0; i < points.Count; i++)
                {
                    heights[i] = defaultWaterLevel;
                }
            }

            return heights;
        }

        /// <summary>
        /// Returns the surface normal at the given world position.
        /// Extension method not in original MARUS interface.
        /// </summary>
        public Vector3 GetSurfaceNormal(Vector3 position)
        {
            if (waterVolume != null)
            {
                return waterVolume.GetSurfaceNormal(position);
            }
            return Vector3.up;
        }

        /// <summary>
        /// Checks if a point is underwater.
        /// Extension method not in original MARUS interface.
        /// </summary>
        public bool IsPointUnderWater(Vector3 position)
        {
            if (waterVolume != null)
            {
                return waterVolume.IsPointUnderWater(position);
            }
            return position.y < defaultWaterLevel;
        }

        /// <summary>
        /// Gets the water density from the WaterVolume.
        /// </summary>
        public float GetWaterDensity()
        {
            if (waterVolume != null)
            {
                return waterVolume.Density;
            }
            return 1000f; // Default freshwater density
        }
    }
}
