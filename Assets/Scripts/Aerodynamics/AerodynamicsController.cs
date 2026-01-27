// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Aerodynamics controller that aggregates forces from multiple surfaces
// Supports both Rigidbody and ArticulationBody

using System.Collections.Generic;
using UnityEngine;

namespace Aerodynamics
{
    /// <summary>
    /// Controller that aggregates aerodynamic forces from multiple surfaces
    /// and applies them to the physics body.
    /// Supports both Rigidbody and ArticulationBody (for URDF robots).
    /// </summary>
    public class AerodynamicsController : MonoBehaviour
    {
        [Header("Physics Body")]
        [SerializeField]
        [Tooltip("If not set, will search for Rigidbody or ArticulationBody")]
        private Rigidbody targetRigidbody;

        [SerializeField]
        [Tooltip("If not set, will search for ArticulationBody")]
        private ArticulationBody targetArticulationBody;

        [Header("Environment")]
        [SerializeField]
        [Tooltip("Enable automatic medium detection for surfaces with Auto mode")]
        private bool enableAutoMediumDetection = true;

        [SerializeField]
        [Tooltip("Default fluid density (kg/m³). Air: 1.225, Seawater: 1027. Set to 0 for auto-detection.")]
        private float defaultFluidDensity = 1.225f;

        [SerializeField]
        [Tooltip("Global wind velocity in world space (m/s)")]
        private Vector3 windVelocity = Vector3.zero;

        [SerializeField]
        [Tooltip("Global underwater current velocity in world space (m/s)")]
        private Vector3 underwaterCurrentVelocity = Vector3.zero;

        [Header("Control")]
        [SerializeField]
        private float pitchSensitivity = 0.3f;

        [SerializeField]
        private float rollSensitivity = 0.3f;

        [SerializeField]
        private float yawSensitivity = 0.3f;

        [SerializeField]
        private float flapSensitivity = 1.0f;

        [Header("Thrust")]
        [SerializeField]
        [Tooltip("Maximum thrust force in Newtons")]
        private float maxThrust = 100f;

        [SerializeField]
        [Tooltip("Thrust direction in local space")]
        private Vector3 thrustDirection = Vector3.forward;

        [SerializeField]
        [Range(0f, 1f)]
        private float thrustPercent = 0f;

        [Header("Prediction (for stability)")]
        [SerializeField]
        [Tooltip("Use velocity prediction for force calculation")]
        private bool usePrediction = true;

        [SerializeField]
        [Range(0f, 1f)]
        private float predictionTimestepFraction = 0.5f;

        [Header("Debug")]
        [SerializeField]
        private bool showDebugInfo = false;

        // Cached surfaces
        private List<AeroSurface> aerodynamicSurfaces = new List<AeroSurface>();
        private BiVector3 currentForceAndTorque;

        // Cached ArticulationBody hierarchy
        private ArticulationBody[] allArticulationBodies;
        private float cachedTotalMass = 0f;
        private bool massNeedsRecalculation = true;

        // Control inputs
        private float pitchInput;
        private float rollInput;
        private float yawInput;
        private float flapInput;

        /// <summary>
        /// Gets the current aggregate force
        /// </summary>
        public Vector3 CurrentForce => currentForceAndTorque.force;

        /// <summary>
        /// Gets the current aggregate torque
        /// </summary>
        public Vector3 CurrentTorque => currentForceAndTorque.torque;

        /// <summary>
        /// Gets or sets the thrust percentage (0-1)
        /// </summary>
        public float ThrustPercent
        {
            get => thrustPercent;
            set => thrustPercent = Mathf.Clamp01(value);
        }

        /// <summary>
        /// Gets or sets the wind velocity
        /// </summary>
        public Vector3 WindVelocity
        {
            get => windVelocity;
            set => windVelocity = value;
        }

        /// <summary>
        /// Gets or sets the fluid density. Set to 0 for auto-detection.
        /// </summary>
        public float FluidDensity
        {
            get => defaultFluidDensity;
            set => defaultFluidDensity = value;
        }

        /// <summary>
        /// Gets or sets whether automatic medium detection is enabled
        /// </summary>
        public bool AutoMediumDetection
        {
            get => enableAutoMediumDetection;
            set => enableAutoMediumDetection = value;
        }

        /// <summary>
        /// Gets or sets the underwater current velocity
        /// </summary>
        public Vector3 UnderwaterCurrentVelocity
        {
            get => underwaterCurrentVelocity;
            set => underwaterCurrentVelocity = value;
        }

        private void Awake()
        {
            // Find physics body - search in children if not on this object
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();

                // If not found on this object, search in children
                if (targetRigidbody == null)
                {
                    targetRigidbody = GetComponentInChildren<Rigidbody>();
                }
            }

            // Find ArticulationBody - search in children if not on this object
            if (targetArticulationBody == null)
            {
                targetArticulationBody = GetComponent<ArticulationBody>();

                // If not found on this object, search in children for the root ArticulationBody
                if (targetArticulationBody == null)
                {
                    var childBodies = GetComponentsInChildren<ArticulationBody>();
                    if (childBodies.Length > 0)
                    {
                        // Find the root ArticulationBody (the one with isRoot = true or no parent)
                        foreach (var body in childBodies)
                        {
                            if (body.isRoot)
                            {
                                targetArticulationBody = body;
                                break;
                            }
                        }

                        // If no explicit root found, use the first one (topmost in hierarchy)
                        if (targetArticulationBody == null)
                        {
                            targetArticulationBody = childBodies[0];
                        }
                    }
                }
            }

            // Cache all ArticulationBodies for total mass calculation
            CacheArticulationBodies();

            // Collect all aerodynamic surfaces in children
            RefreshSurfaces();

            // Debug: Log which physics body is being used
            if (showDebugInfo)
            {
                if (targetRigidbody != null)
                {
                    Debug.Log($"[AerodynamicsController] Using Rigidbody: {targetRigidbody.gameObject.name}");
                }
                else if (targetArticulationBody != null)
                {
                    Debug.Log($"[AerodynamicsController] Using ArticulationBody: {targetArticulationBody.gameObject.name} (root: {targetArticulationBody.isRoot})");
                }
                else
                {
                    Debug.LogWarning("[AerodynamicsController] No physics body found! Aerodynamics will not work.");
                }
            }
        }

        /// <summary>
        /// Caches all ArticulationBodies in the hierarchy for mass calculation
        /// </summary>
        private void CacheArticulationBodies()
        {
            if (targetArticulationBody != null)
            {
                // Get all ArticulationBodies starting from the root
                ArticulationBody rootBody = targetArticulationBody;

                // Find the actual root if this isn't it
                while (rootBody.transform.parent != null)
                {
                    var parentBody = rootBody.transform.parent.GetComponent<ArticulationBody>();
                    if (parentBody != null)
                    {
                        rootBody = parentBody;
                    }
                    else
                    {
                        break;
                    }
                }

                // Get all bodies from root downward
                allArticulationBodies = rootBody.GetComponentsInChildren<ArticulationBody>();
                massNeedsRecalculation = true;

                if (showDebugInfo)
                {
                    Debug.Log($"[AerodynamicsController] Cached {allArticulationBodies.Length} ArticulationBodies");
                }
            }
            else
            {
                allArticulationBodies = null;
            }
        }

        /// <summary>
        /// Refreshes the list of aerodynamic surfaces
        /// </summary>
        public void RefreshSurfaces()
        {
            aerodynamicSurfaces.Clear();
            aerodynamicSurfaces.AddRange(GetComponentsInChildren<AeroSurface>());

            if (showDebugInfo)
            {
                Debug.Log($"[AerodynamicsController] Found {aerodynamicSurfaces.Count} aerodynamic surfaces");
            }
        }

        /// <summary>
        /// Adds an aerodynamic surface to the controller
        /// </summary>
        public void AddSurface(AeroSurface surface)
        {
            if (!aerodynamicSurfaces.Contains(surface))
            {
                aerodynamicSurfaces.Add(surface);
            }
        }

        /// <summary>
        /// Sets control surface inputs
        /// </summary>
        /// <param name="pitch">Pitch input (-1 to 1)</param>
        /// <param name="roll">Roll input (-1 to 1)</param>
        /// <param name="yaw">Yaw input (-1 to 1)</param>
        /// <param name="flap">Flap input (0 to 1)</param>
        public void SetControlInputs(float pitch, float roll, float yaw, float flap)
        {
            pitchInput = Mathf.Clamp(pitch, -1f, 1f);
            rollInput = Mathf.Clamp(roll, -1f, 1f);
            yawInput = Mathf.Clamp(yaw, -1f, 1f);
            flapInput = Mathf.Clamp01(flap);

            ApplyControlInputsToSurfaces();
        }

        private void ApplyControlInputsToSurfaces()
        {
            foreach (var surface in aerodynamicSurfaces)
            {
                if (!surface.IsControlSurface) continue;

                float angle = 0f;
                switch (surface.InputType)
                {
                    case ControlInputType.Pitch:
                        angle = pitchInput * pitchSensitivity * surface.InputMultiplier;
                        break;
                    case ControlInputType.Roll:
                        angle = rollInput * rollSensitivity * surface.InputMultiplier;
                        break;
                    case ControlInputType.Yaw:
                        angle = yawInput * yawSensitivity * surface.InputMultiplier;
                        break;
                    case ControlInputType.Flap:
                        angle = flapInput * flapSensitivity * surface.InputMultiplier;
                        break;
                }

                surface.SetFlapAngle(angle);
            }
        }

        private void FixedUpdate()
        {
            if (targetRigidbody == null && targetArticulationBody == null)
            {
                return;
            }

            Vector3 velocity = GetVelocity();
            Vector3 angularVelocity = GetAngularVelocity();
            Vector3 centerOfMass = GetCenterOfMass();

            // Calculate aerodynamic forces
            List<SurfaceForce> surfaceForces;
            if (usePrediction)
            {
                surfaceForces = CalculateAerodynamicForcesWithPrediction(
                    velocity, angularVelocity, centerOfMass, out currentForceAndTorque);
            }
            else
            {
                surfaceForces = ComputeSurfaceForces(
                    velocity, angularVelocity, centerOfMass, out currentForceAndTorque);
            }

            // Apply forces per surface body
            ApplySurfaceForces(surfaceForces);

            // Apply thrust
            if (thrustPercent > 0f)
            {
                Vector3 thrustForce = transform.TransformDirection(thrustDirection.normalized) *
                                     maxThrust * thrustPercent;
                ApplyForce(thrustForce);
            }
        }

        private struct SurfaceForce
        {
            public AeroSurface surface;
            public ArticulationBody articulationBody;
            public Rigidbody rigidbody;
            public BiVector3 forces;
        }

        private List<SurfaceForce> ComputeSurfaceForces(
            Vector3 defaultVelocity,
            Vector3 defaultAngularVelocity,
            Vector3 defaultCenterOfMass,
            out BiVector3 totalForceAndTorque)
        {
            totalForceAndTorque = BiVector3.zero;
            var results = new List<SurfaceForce>(aerodynamicSurfaces.Count);

            foreach (var surface in aerodynamicSurfaces)
            {
                if (surface == null) continue;

                ArticulationBody surfaceArticulation = surface.GetComponentInParent<ArticulationBody>();
                Rigidbody surfaceRigidbody = surface.GetComponentInParent<Rigidbody>();

                Vector3 surfaceVelocity;
                Vector3 bodyAngularVelocity;
                Vector3 bodyCenterOfMass;

                if (surfaceArticulation != null)
                {
                    surfaceVelocity = surfaceArticulation.linearVelocity;
                    bodyAngularVelocity = surfaceArticulation.angularVelocity;
                    bodyCenterOfMass = surfaceArticulation.worldCenterOfMass;
                }
                else if (surfaceRigidbody != null)
                {
                    surfaceVelocity = surfaceRigidbody.linearVelocity;
                    bodyAngularVelocity = surfaceRigidbody.angularVelocity;
                    bodyCenterOfMass = surfaceRigidbody.worldCenterOfMass;
                }
                else
                {
                    surfaceVelocity = defaultVelocity;
                    bodyAngularVelocity = defaultAngularVelocity;
                    bodyCenterOfMass = defaultCenterOfMass;
                }
                Vector3 relativePosition = surface.transform.position - bodyCenterOfMass;
                Debug.Log("Name:" + surface.name + " surfaceVelocity: " + surfaceVelocity);
                Debug.Log("bodyAngularVelocity: " + bodyAngularVelocity);

                BiVector3 surfaceForces;

                // Check if we should use auto-detection for this surface
                bool useAutoDetection = enableAutoMediumDetection &&
                    surface.Parameters.fluidMedium == FluidMedium.Auto;

                if (useAutoDetection)
                {
                    // Use auto-detection mode
                    // Determine ambient fluid velocity based on detected medium
                    Vector3 ambientFluidVelocity;
                    switch (surface.CurrentMedium)
                    {
                        case CurrentMedium.Water:
                            ambientFluidVelocity = underwaterCurrentVelocity;
                            break;
                        case CurrentMedium.Transition:
                            // Interpolate between wind and current based on submersion
                            ambientFluidVelocity = Vector3.Lerp(
                                windVelocity,
                                underwaterCurrentVelocity,
                                surface.SubmersionRatio);
                            break;
                        default:
                            ambientFluidVelocity = windVelocity;
                            break;
                    }

                    // Fluid velocity relative to surface
                    Vector3 fluidVelocity = -surfaceVelocity + ambientFluidVelocity;
                    // Pass angular velocity for per-element velocity calculation in BET
                    surfaceForces = surface.CalculateForcesAutoMedium(fluidVelocity, relativePosition, bodyAngularVelocity);
                }
                else
                {
                    // Use manual density mode
                    // Determine ambient velocity based on configured medium
                    Vector3 ambientFluidVelocity = surface.Parameters.fluidMedium == FluidMedium.Water
                        ? underwaterCurrentVelocity
                        : windVelocity;

                    // Fluid velocity relative to surface
                    Vector3 fluidVelocity = -surfaceVelocity + ambientFluidVelocity;
                    // Pass angular velocity for per-element velocity calculation in BET
                    surfaceForces = surface.CalculateForces(
                        fluidVelocity,
                        defaultFluidDensity,
                        relativePosition,
                        bodyAngularVelocity);
                }

                totalForceAndTorque += surfaceForces;
                results.Add(new SurfaceForce
                {
                    surface = surface,
                    articulationBody = surfaceArticulation,
                    rigidbody = surfaceRigidbody,
                    forces = surfaceForces
                });
            }

            return results;
        }

        private List<SurfaceForce> AverageSurfaceForces(
            List<SurfaceForce> current,
            List<SurfaceForce> predicted,
            out BiVector3 totalForceAndTorque)
        {
            totalForceAndTorque = BiVector3.zero;
            if (current == null || predicted == null || current.Count != predicted.Count)
            {
                return current ?? new List<SurfaceForce>();
            }

            var averaged = new List<SurfaceForce>(current.Count);
            for (int i = 0; i < current.Count; i++)
            {
                SurfaceForce avg = current[i];
                avg.forces = (current[i].forces + predicted[i].forces) * 0.5f;
                totalForceAndTorque += avg.forces;
                averaged.Add(avg);
            }

            return averaged;
        }

        private List<SurfaceForce> CalculateAerodynamicForcesWithPrediction(
            Vector3 velocity,
            Vector3 angularVelocity,
            Vector3 centerOfMass,
            out BiVector3 totalForceAndTorque)
        {
            // Calculate forces at current state
            BiVector3 currentTotal;
            List<SurfaceForce> currentForces = ComputeSurfaceForces(
                velocity, angularVelocity, centerOfMass, out currentTotal);

            // Get thrust force
            Vector3 thrustForce = transform.TransformDirection(thrustDirection.normalized) *
                                 maxThrust * thrustPercent;

            // Predict velocity at next timestep (root-level approximation)
            float mass = GetMass();
            Vector3 totalForce = currentTotal.force + thrustForce + Physics.gravity * mass;
            Vector3 predictedVelocity = velocity +
                Time.fixedDeltaTime * predictionTimestepFraction * totalForce / mass;

            // Calculate forces at predicted state
            BiVector3 predictedTotal;
            List<SurfaceForce> predictedForces = ComputeSurfaceForces(
                predictedVelocity, angularVelocity, centerOfMass, out predictedTotal);

            // Average current and predicted forces
            return AverageSurfaceForces(currentForces, predictedForces, out totalForceAndTorque);
        }

        private Vector3 GetVelocity()
        {
            if (targetRigidbody != null)
                return targetRigidbody.linearVelocity;
            if (targetArticulationBody != null)
                return targetArticulationBody.linearVelocity;
            return Vector3.zero;
        }

        private Vector3 GetAngularVelocity()
        {
            if (targetRigidbody != null)
                return targetRigidbody.angularVelocity;
            if (targetArticulationBody != null)
                return targetArticulationBody.angularVelocity;
            return Vector3.zero;
        }

        private Vector3 GetCenterOfMass()
        {
            if (targetRigidbody != null)
                return targetRigidbody.worldCenterOfMass;
            if (targetArticulationBody != null)
                return targetArticulationBody.worldCenterOfMass;
            return transform.position;
        }

        private float GetMass()
        {
            if (targetRigidbody != null)
                return targetRigidbody.mass;

            // For ArticulationBody, return the total mass of the entire robot
            if (targetArticulationBody != null)
            {
                return GetTotalArticulationMass();
            }

            return 1f;
        }

        /// <summary>
        /// Calculates the total mass of all ArticulationBodies in the robot hierarchy
        /// </summary>
        private float GetTotalArticulationMass()
        {
            // Return cached value if available
            if (!massNeedsRecalculation && cachedTotalMass > 0f)
            {
                return cachedTotalMass;
            }

            // Recalculate total mass
            if (allArticulationBodies != null && allArticulationBodies.Length > 0)
            {
                cachedTotalMass = 0f;
                foreach (var body in allArticulationBodies)
                {
                    if (body != null)
                    {
                        cachedTotalMass += body.mass;
                    }
                }
                massNeedsRecalculation = false;

                if (showDebugInfo)
                {
                    Debug.Log($"[AerodynamicsController] Total robot mass: {cachedTotalMass} kg ({allArticulationBodies.Length} bodies)");
                }

                return cachedTotalMass;
            }

            // Fallback to single body mass
            return targetArticulationBody != null ? targetArticulationBody.mass : 1f;
        }

        /// <summary>
        /// Forces recalculation of total mass on next GetMass() call
        /// Call this if robot structure changes at runtime
        /// </summary>
        public void InvalidateMassCache()
        {
            massNeedsRecalculation = true;
            CacheArticulationBodies();
        }

        private void ApplyForces(BiVector3 forceAndTorque)
        {
            if (targetRigidbody != null)
            {
                targetRigidbody.AddForce(forceAndTorque.force);
                targetRigidbody.AddTorque(forceAndTorque.torque);
            }
            else if (targetArticulationBody != null)
            {
                targetArticulationBody.AddForce(forceAndTorque.force);
                targetArticulationBody.AddTorque(forceAndTorque.torque);
            }
        }

        private void ApplySurfaceForces(List<SurfaceForce> surfaceForces)
        {
            if (surfaceForces == null) return;

            foreach (var surfaceForce in surfaceForces)
            {
                if (surfaceForce.articulationBody != null)
                {
                    Debug.Log("Name: " + surfaceForce.articulationBody.name + " Applying force: " + surfaceForce.forces.force);
                    surfaceForce.articulationBody.AddForce(surfaceForce.forces.force);
                    Debug.Log("Applying torque: " + surfaceForce.forces.torque);
                    surfaceForce.articulationBody.AddTorque(surfaceForce.forces.torque);
                }
                else if (surfaceForce.rigidbody != null)
                {
                    surfaceForce.rigidbody.AddForce(surfaceForce.forces.force);
                    surfaceForce.rigidbody.AddTorque(surfaceForce.forces.torque);
                }
                else
                {
                    ApplyForces(surfaceForce.forces);
                }
            }
        }

        private void ApplyForce(Vector3 force)
        {
            if (targetRigidbody != null)
            {
                targetRigidbody.AddForce(force);
            }
            else if (targetArticulationBody != null)
            {
                targetArticulationBody.AddForce(force);
            }
        }

        private void OnDrawGizmosSelected()
        {
            if (!showDebugInfo) return;

            // Draw wind direction
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, windVelocity);

            // Draw thrust direction
            Gizmos.color = Color.red;
            Gizmos.DrawRay(transform.position,
                transform.TransformDirection(thrustDirection.normalized) * maxThrust * 0.01f);
        }
    }
}
