// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Extended ArticulationFloatingObject with MARUS hydrodynamic forces
// Combines NaughtyWaterBuoyancy's buoyancy with MARUS's hydrodynamic modeling

using System.Collections.Generic;
using UnityEngine;
using NaughtyWaterBuoyancy;

namespace Hydrodynamics
{
    /// <summary>
    /// Extended floating object that adds MARUS hydrodynamic forces to ArticulationBody robots.
    /// Implements: Buoyancy, Viscous Water Resistance, Pressure Drag, Slamming Force, Air Resistance
    /// </summary>
    [RequireComponent(typeof(Collider))]
    [RequireComponent(typeof(ArticulationBody))]
    public class HydrodynamicFloatingObject : MonoBehaviour
    {
        [Header("Basic Properties")]
        [SerializeField]
        [Tooltip("Object density in kg/m^3")]
        private float density = 1.0f;

        [SerializeField]
        [Range(0f, 1f)]
        [Tooltip("Normalized voxel size for buoyancy calculation")]
        private float normalizedVoxelSize = 0.5f;

        [Header("Damping")]
        [SerializeField]
        private float linearDampingInWater = 1f;

        [SerializeField]
        private float angularDampingInWater = 1f;

        [Header("Hydrodynamic Parameters")]
        [SerializeField]
        [Tooltip("Use ScriptableObject config or inline parameters")]
        private bool useConfigAsset = false;

        [SerializeField]
        private HydrodynamicConfig configAsset;

        [SerializeField]
        private HydrodynamicParameters parameters = new HydrodynamicParameters();

        [Header("Debug")]
        [SerializeField]
        private bool showDebugForces = false;

        [SerializeField]
        private bool showDebugVoxels = false;

        // Component references
        private WaterVolume water;
        private Collider objectCollider;
        private ArticulationBody articulationBody;

        // State tracking
        private float initialLinearDamping;
        private float initialAngularDamping;
        private Vector3 voxelSize;
        private Vector3[] voxels;
        private float objectVolume;

        // Velocity tracking for slamming force
        private Vector3[] previousVoxelVelocities;
        private float[] previousSubmergedFactors;

        // Debug visualization
        private List<(Vector3 position, Vector3 force, Color color)> debugForces = new List<(Vector3, Vector3, Color)>();

        // Properties
        public float Density
        {
            get { return this.density; }
            set { this.density = value; }
        }

        public HydrodynamicParameters Parameters
        {
            get
            {
                if (useConfigAsset && configAsset != null)
                    return configAsset.parameters;
                return parameters;
            }
            set
            {
                parameters = value;
                useConfigAsset = false; // インラインパラメータを使用
            }
        }

        protected virtual void Awake()
        {
            this.objectCollider = this.GetComponent<Collider>();
            this.articulationBody = this.GetComponent<ArticulationBody>();

            this.initialLinearDamping = this.articulationBody.linearDamping;
            this.initialAngularDamping = this.articulationBody.angularDamping;

            // Calculate volume from mesh if available
            MeshFilter meshFilter = this.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
            {
                this.objectVolume = MathfUtils.CalculateVolume_Mesh(meshFilter.sharedMesh, this.transform);
            }
            else
            {
                // Estimate volume from collider bounds
                Bounds bounds = this.objectCollider.bounds;
                this.objectVolume = bounds.size.x * bounds.size.y * bounds.size.z;
            }
        }

        protected virtual void FixedUpdate()
        {
            if (this.water == null || this.voxels == null || this.voxels.Length == 0)
                return;

            if (showDebugForces)
                debugForces.Clear();

            var p = Parameters;
            Vector3 maxBuoyancyForce = CalculateMaxBuoyancyForce();
            Vector3 forceAtSingleVoxel = maxBuoyancyForce / this.voxels.Length;
            Bounds bounds = this.objectCollider.bounds;
            float voxelHeight = bounds.size.y * this.normalizedVoxelSize;

            // Initialize velocity tracking if needed
            if (previousVoxelVelocities == null || previousVoxelVelocities.Length != voxels.Length)
            {
                previousVoxelVelocities = new Vector3[voxels.Length];
                previousSubmergedFactors = new float[voxels.Length];
            }

            float totalSubmergedFactor = 0f;
            float totalAboveWaterFactor = 0f;

            // Calculate resistance coefficient (MARUS formula)
            float velocity = articulationBody.linearVelocity.magnitude;
            float characteristicLength = Mathf.Max(bounds.size.x, bounds.size.z);
            float Cf = CalculateResistanceCoefficient(p.waterDensity, velocity, characteristicLength, p.waterViscosity);

            for (int i = 0; i < this.voxels.Length; i++)
            {
                Vector3 worldPoint = this.transform.TransformPoint(this.voxels[i]);
                float waterLevel = this.water.GetWaterLevel(worldPoint);
                float deepLevel = waterLevel - worldPoint.y + (voxelHeight / 2f);
                float submergedFactor = Mathf.Clamp(deepLevel / voxelHeight, 0f, 1f);

                Vector3 voxelVelocity = articulationBody.GetPointVelocity(worldPoint);
                Vector3 totalForce = Vector3.zero;

                if (submergedFactor > 0f)
                {
                    // === UNDERWATER FORCES ===
                    totalSubmergedFactor += submergedFactor;

                    // 1. Buoyancy Force (NaughtyWaterBuoyancy style with surface normal adjustment)
                    Vector3 surfaceNormal = this.water.GetSurfaceNormal(worldPoint);
                    Quaternion surfaceRotation = Quaternion.FromToRotation(this.water.transform.up, surfaceNormal);
                    surfaceRotation = Quaternion.Slerp(surfaceRotation, Quaternion.identity, submergedFactor);
                    Vector3 buoyancyForce = surfaceRotation * (forceAtSingleVoxel * submergedFactor);
                    totalForce += buoyancyForce;

                    if (showDebugForces)
                        debugForces.Add((worldPoint, buoyancyForce * 0.001f, Color.blue));

                    // 2. Viscous Water Resistance (MARUS formula)
                    if (p.enableViscousResistance && voxelVelocity.sqrMagnitude > 0.0001f)
                    {
                        Vector3 viscousForce = CalculateViscousResistance(
                            p.waterDensity, voxelVelocity, surfaceNormal,
                            voxelSize.x * voxelSize.z, Cf);
                        totalForce += viscousForce * submergedFactor;

                        if (showDebugForces)
                            debugForces.Add((worldPoint, viscousForce * 0.01f, Color.green));
                    }

                    // 3. Pressure Drag Force (MARUS formula)
                    if (p.enablePressureDrag && voxelVelocity.sqrMagnitude > 0.0001f)
                    {
                        Vector3 pressureForce = CalculatePressureDrag(
                            voxelVelocity, surfaceNormal,
                            voxelSize.x * voxelSize.z, p);
                        totalForce += pressureForce * submergedFactor;

                        if (showDebugForces)
                            debugForces.Add((worldPoint, pressureForce * 0.01f, Color.yellow));
                    }

                    // 4. Slamming Force (MARUS formula)
                    if (p.enableSlammingForce)
                    {
                        Vector3 slammingForce = CalculateSlammingForce(
                            i, voxelVelocity, surfaceNormal,
                            submergedFactor, voxelSize.x * voxelSize.z, p);
                        totalForce += slammingForce;

                        if (showDebugForces && slammingForce.sqrMagnitude > 0.1f)
                            debugForces.Add((worldPoint, slammingForce * 0.001f, Color.red));
                    }
                }
                else
                {
                    // === ABOVE WATER FORCES ===
                    totalAboveWaterFactor += 1f;

                    // Air Resistance (MARUS formula)
                    if (p.enableAirResistance && voxelVelocity.sqrMagnitude > 0.0001f)
                    {
                        Vector3 airForce = CalculateAirResistance(
                            p.airDensity, voxelVelocity,
                            voxelSize.x * voxelSize.z, p.airResistanceCoefficient);
                        totalForce += airForce;

                        if (showDebugForces)
                            debugForces.Add((worldPoint, airForce * 0.1f, Color.cyan));
                    }
                }

                // Apply combined force
                if (totalForce.sqrMagnitude > 0.0001f)
                {
                    articulationBody.AddForceAtPosition(totalForce, worldPoint);
                }

                // Store for next frame
                previousVoxelVelocities[i] = voxelVelocity;
                previousSubmergedFactors[i] = submergedFactor;
            }

            // Update damping based on submersion
            float normalizedSubmersion = totalSubmergedFactor / this.voxels.Length;
            this.articulationBody.linearDamping = Mathf.Lerp(
                this.initialLinearDamping, this.linearDampingInWater, normalizedSubmersion);
            this.articulationBody.angularDamping = Mathf.Lerp(
                this.initialAngularDamping, this.angularDampingInWater, normalizedSubmersion);
        }

        #region Force Calculations

        private Vector3 CalculateMaxBuoyancyForce()
        {
            float volume = this.articulationBody.mass / this.density;
            float waterDensity = this.water != null ? this.water.Density : Parameters.waterDensity;
            return waterDensity * volume * -Physics.gravity;
        }

        /// <summary>
        /// Calculates the frictional resistance coefficient based on Reynolds number
        /// </summary>
        private float CalculateResistanceCoefficient(float rho, float velocity, float length, float viscosity)
        {
            if (velocity < 0.001f || length < 0.001f)
                return 0f;

            float Rn = (velocity * length) / viscosity;
            if (Rn < 1f) return 0f;

            // ITTC 1957 friction line
            float Cf = 0.075f / Mathf.Pow((Mathf.Log10(Rn) - 2f), 2f);
            return Cf;
        }

        /// <summary>
        /// Viscous water resistance - based on MARUS BoatPhysicsMath.ViscousWaterResistanceForce
        /// </summary>
        private Vector3 CalculateViscousResistance(float rho, Vector3 velocity, Vector3 normal, float area, float Cf)
        {
            // Get tangential velocity component
            Vector3 velocityTangent = Vector3.Cross(normal, Vector3.Cross(velocity, normal) / normal.magnitude) / normal.magnitude;
            Vector3 tangentialDirection = velocityTangent.normalized * -1f;

            // Flow velocity in tangent direction
            Vector3 v_f_vec = velocity.magnitude * tangentialDirection;

            // F = 0.5 * rho * v^2 * S * Cf
            Vector3 viscousForce = 0.5f * rho * v_f_vec.magnitude * v_f_vec * area * Cf;

            return CheckForceValid(viscousForce);
        }

        /// <summary>
        /// Pressure drag force - based on MARUS BoatPhysicsMath.PressureDragForce
        /// </summary>
        private Vector3 CalculatePressureDrag(Vector3 velocity, Vector3 normal, float area, HydrodynamicParameters p)
        {
            float speed = velocity.magnitude;
            float cosTheta = Vector3.Dot(velocity.normalized, normal);
            float normalizedVelocity = speed / p.velocityReference;

            Vector3 pressureForce = Vector3.zero;

            if (cosTheta > 0f)
            {
                // Facing the flow - pressure drag
                pressureForce = -(p.C_PD1 * normalizedVelocity + p.C_PD2 * normalizedVelocity * normalizedVelocity)
                    * area * Mathf.Pow(cosTheta, p.f_P) * normal;
            }
            else
            {
                // Away from flow - suction drag
                pressureForce = (p.C_SD1 * normalizedVelocity + p.C_SD2 * normalizedVelocity * normalizedVelocity)
                    * area * Mathf.Pow(Mathf.Abs(cosTheta), p.f_S) * normal;
            }

            return CheckForceValid(pressureForce);
        }

        /// <summary>
        /// Slamming force for water entry - based on MARUS BoatPhysicsMath.SlammingForce
        /// </summary>
        private Vector3 CalculateSlammingForce(int voxelIndex, Vector3 velocity, Vector3 normal,
            float submergedFactor, float area, HydrodynamicParameters p)
        {
            float cosTheta = Vector3.Dot(velocity.normalized, normal);

            // Only apply slamming when entering water (normal facing flow direction)
            if (cosTheta <= 0f)
                return Vector3.zero;

            // Calculate acceleration from velocity change
            Vector3 previousVelocity = previousVoxelVelocities[voxelIndex];
            float previousSubmerged = previousSubmergedFactors[voxelIndex];

            // Only apply slamming when transitioning into water
            if (submergedFactor <= previousSubmerged)
                return Vector3.zero;

            Vector3 acceleration = (velocity - previousVelocity) / Time.fixedDeltaTime;
            float accMagnitude = acceleration.magnitude;

            // F = clamp(acc/acc_max)^p * cos(theta) * F_stop * multiplier
            // F_stop approximation based on momentum
            float F_stop_magnitude = articulationBody.mass * velocity.magnitude * (2f * area / (objectVolume > 0 ? objectVolume : 1f));
            Vector3 F_stop = velocity.normalized * F_stop_magnitude;

            float slammingMagnitude = Mathf.Pow(Mathf.Clamp01(accMagnitude / p.maxAcceleration), p.slammingPower)
                * cosTheta * p.slammingMultiplier;

            Vector3 slammingForce = -F_stop * slammingMagnitude;

            return CheckForceValid(slammingForce);
        }

        /// <summary>
        /// Air resistance - based on MARUS BoatPhysicsMath.AirResistanceForce
        /// </summary>
        private Vector3 CalculateAirResistance(float rho, Vector3 velocity, float area, float Cd)
        {
            // R_air = 0.5 * rho * v^2 * A * C_air
            Vector3 airForce = 0.5f * rho * velocity.magnitude * velocity * area * Cd;
            return CheckForceValid(-airForce); // Opposite to velocity
        }

        private Vector3 CheckForceValid(Vector3 force)
        {
            if (float.IsNaN(force.x) || float.IsNaN(force.y) || float.IsNaN(force.z))
                return Vector3.zero;
            if (float.IsInfinity(force.x) || float.IsInfinity(force.y) || float.IsInfinity(force.z))
                return Vector3.zero;
            return force;
        }

        #endregion

        #region Water Detection

        protected virtual void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag(WaterVolume.TAG))
            {
                this.water = other.GetComponent<WaterVolume>();
                if (this.voxels == null)
                {
                    this.voxels = this.CutIntoVoxels();
                }
            }
        }

        protected virtual void OnTriggerStay(Collider other)
        {
            if (other.CompareTag(WaterVolume.TAG) && this.water == null)
            {
                this.water = other.GetComponent<WaterVolume>();
                if (this.voxels == null)
                {
                    this.voxels = this.CutIntoVoxels();
                }
            }
        }

        protected virtual void OnTriggerExit(Collider other)
        {
            if (other.CompareTag(WaterVolume.TAG) && this.water != null)
            {
                Bounds bounds = this.objectCollider.bounds;
                float lowestPoint = bounds.min.y;
                float waterLevel = this.water.GetWaterLevel(this.transform.position);

                if (lowestPoint > waterLevel + 0.1f)
                {
                    this.water = null;
                }
            }
        }

        #endregion

        #region Voxelization

        private Vector3[] CutIntoVoxels()
        {
            Quaternion initialRotation = this.transform.rotation;
            this.transform.rotation = Quaternion.identity;

            Bounds bounds = this.objectCollider.bounds;
            this.voxelSize.x = bounds.size.x * this.normalizedVoxelSize;
            this.voxelSize.y = bounds.size.y * this.normalizedVoxelSize;
            this.voxelSize.z = bounds.size.z * this.normalizedVoxelSize;

            if (this.voxelSize.x <= 0 || this.voxelSize.y <= 0 || this.voxelSize.z <= 0)
            {
                this.transform.rotation = initialRotation;
                return new Vector3[0];
            }

            int voxelsCountForEachAxis = Mathf.RoundToInt(1f / this.normalizedVoxelSize);
            List<Vector3> voxelList = new List<Vector3>(voxelsCountForEachAxis * voxelsCountForEachAxis * voxelsCountForEachAxis);

            for (int i = 0; i < voxelsCountForEachAxis; i++)
            {
                for (int j = 0; j < voxelsCountForEachAxis; j++)
                {
                    for (int k = 0; k < voxelsCountForEachAxis; k++)
                    {
                        float pX = bounds.min.x + this.voxelSize.x * (0.5f + i);
                        float pY = bounds.min.y + this.voxelSize.y * (0.5f + j);
                        float pZ = bounds.min.z + this.voxelSize.z * (0.5f + k);

                        Vector3 point = new Vector3(pX, pY, pZ);
                        if (ColliderUtils.IsPointInsideCollider(point, this.objectCollider, ref bounds))
                        {
                            voxelList.Add(this.transform.InverseTransformPoint(point));
                        }
                    }
                }
            }

            this.transform.rotation = initialRotation;
            return voxelList.ToArray();
        }

        #endregion

        #region Debug Visualization

        protected virtual void OnDrawGizmos()
        {
            if (showDebugVoxels && this.voxels != null)
            {
                for (int i = 0; i < this.voxels.Length; i++)
                {
                    Gizmos.color = Color.cyan - new Color(0f, 0f, 0f, 0.75f);
                    Gizmos.DrawCube(this.transform.TransformPoint(this.voxels[i]), this.voxelSize * 0.8f);
                }
            }

            if (showDebugForces)
            {
                foreach (var (position, force, color) in debugForces)
                {
                    Gizmos.color = color;
                    Gizmos.DrawLine(position, position + force);
                    Gizmos.DrawSphere(position, 0.02f);
                }
            }
        }

        #endregion
    }
}
