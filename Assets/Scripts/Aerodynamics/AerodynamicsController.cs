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
            // Find physics body
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }
            if (targetArticulationBody == null)
            {
                targetArticulationBody = GetComponent<ArticulationBody>();
            }

            // Collect all aerodynamic surfaces in children
            RefreshSurfaces();
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
            if (usePrediction)
            {
                currentForceAndTorque = CalculateAerodynamicForcesWithPrediction(
                    velocity, angularVelocity, centerOfMass);
            }
            else
            {
                currentForceAndTorque = CalculateAerodynamicForces(
                    velocity, angularVelocity, centerOfMass);
            }

            // Apply forces
            ApplyForces(currentForceAndTorque);

            // Apply thrust
            if (thrustPercent > 0f)
            {
                Vector3 thrustForce = transform.TransformDirection(thrustDirection.normalized) *
                                     maxThrust * thrustPercent;
                ApplyForce(thrustForce);
            }
        }

        private BiVector3 CalculateAerodynamicForces(
            Vector3 velocity,
            Vector3 angularVelocity,
            Vector3 centerOfMass)
        {
            BiVector3 totalForceAndTorque = BiVector3.zero;

            foreach (var surface in aerodynamicSurfaces)
            {
                if (surface == null) continue;

                Vector3 relativePosition = surface.transform.position - centerOfMass;

                // Calculate local velocity at this surface
                // Account for rotational velocity: v_local = v_body - ω × r
                Vector3 surfaceVelocity = velocity + Vector3.Cross(angularVelocity, relativePosition);

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
                    surfaceForces = surface.CalculateForcesAutoMedium(fluidVelocity, relativePosition);
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
                    surfaceForces = surface.CalculateForces(
                        fluidVelocity,
                        defaultFluidDensity,
                        relativePosition);
                }

                totalForceAndTorque += surfaceForces;
            }

            return totalForceAndTorque;
        }

        private BiVector3 CalculateAerodynamicForcesWithPrediction(
            Vector3 velocity,
            Vector3 angularVelocity,
            Vector3 centerOfMass)
        {
            // Calculate forces at current state
            BiVector3 currentForces = CalculateAerodynamicForces(velocity, angularVelocity, centerOfMass);

            // Get thrust force
            Vector3 thrustForce = transform.TransformDirection(thrustDirection.normalized) *
                                 maxThrust * thrustPercent;

            // Predict velocity at next timestep
            float mass = GetMass();
            Vector3 totalForce = currentForces.force + thrustForce + Physics.gravity * mass;
            Vector3 predictedVelocity = velocity +
                Time.fixedDeltaTime * predictionTimestepFraction * totalForce / mass;

            // Calculate forces at predicted state
            BiVector3 predictedForces = CalculateAerodynamicForces(
                predictedVelocity, angularVelocity, centerOfMass);

            // Average current and predicted forces
            return (currentForces + predictedForces) * 0.5f;
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
            if (targetArticulationBody != null)
                return targetArticulationBody.mass;
            return 1f;
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
