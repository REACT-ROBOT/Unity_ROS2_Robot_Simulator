// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Aerodynamics controller that aggregates forces from multiple surfaces
// Supports both Rigidbody and ArticulationBody

using System.Collections.Generic;
using System.IO;
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

        [Header("Surface Velocity Filtering")]
        [SerializeField]
        [Tooltip("Enable filtering to smooth surface velocity used for lift/drag")]
        private bool enableSurfaceVelocityFiltering = true;

        [SerializeField]
        [Tooltip("Time constant (seconds) for surface velocity smoothing. 0 disables smoothing.")]
        private float surfaceVelocityFilterTime = 0.1f;

        [SerializeField]
        [Tooltip("Clamp surface velocity magnitude (m/s) when using FullVector clamp. 0 disables.")]
        private float maxSurfaceSpeed = 100.0f;

        [Header("Vertical Lift Smoothing")]
        [SerializeField]
        [Tooltip("Enable smoothing for vertical (Y) component of aerodynamic force")]
        private bool enableVerticalForceSmoothing = true;

        [SerializeField]
        [Tooltip("Time constant (seconds) for vertical force smoothing. 0 disables smoothing.")]
        private float verticalForceFilterTime = 0.1f;

        [SerializeField]
        [Tooltip("Clamp upward force per surface (N). 0 disables clamp.")]
        private float maxUpwardForcePerSurface = 0f;

        [Header("Angular Velocity Filtering")]
        [SerializeField]
        [Tooltip("Enable filtering to smooth angular velocity used for force calculation")]
        private bool enableAngularVelocityFiltering = true;

        [SerializeField]
        [Tooltip("Time constant (seconds) for angular velocity smoothing. 0 disables smoothing.")]
        private float angularVelocityFilterTime = 0.1f;

        [SerializeField]
        [Tooltip("Clamp angular velocity magnitude (rad/s). 0 disables clamp.")]
        private float maxAngularSpeed = 10f;

        [Header("Debug")]
        [SerializeField]
        private bool showDebugInfo = false;

        [SerializeField]
        [Tooltip("Enable file logging of aerodynamic data")]
        private bool enableFileLogging = false;

        [SerializeField]
        [Tooltip("Log file path (default: ~/aero_debug.csv)")]
        private string logFilePath = "";

        // File logging
        private StreamWriter logWriter;
        private float simulationTime = 0f;
        private bool logHeaderWritten = false;

        // Cached surfaces
        private List<AeroSurface> aerodynamicSurfaces = new List<AeroSurface>();
        private BiVector3 currentForceAndTorque;
        private readonly Dictionary<AeroSurface, Vector3> filteredSurfaceVelocities =
            new Dictionary<AeroSurface, Vector3>();
        private readonly Dictionary<AeroSurface, float> filteredSurfaceVerticalForces =
            new Dictionary<AeroSurface, float>();
        private readonly Dictionary<AeroSurface, Vector3> filteredSurfaceAngularVelocities =
            new Dictionary<AeroSurface, Vector3>();

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

            // Initialize file logging
            InitializeFileLogging();
        }

        private void OnDestroy()
        {
            CloseFileLogging();
        }

        private void OnDisable()
        {
            CloseFileLogging();
        }

        private void InitializeFileLogging()
        {
            if (!enableFileLogging) return;

            try
            {
                string path = logFilePath;
                if (string.IsNullOrEmpty(path))
                {
                    path = Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.UserProfile), "aero_debug.csv");
                }

                logWriter = new StreamWriter(path, false);
                logHeaderWritten = false;
                simulationTime = 0f;
                Debug.Log($"[AerodynamicsController] File logging enabled: {path}");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[AerodynamicsController] Failed to open log file: {e.Message}");
                enableFileLogging = false;
            }
        }

        private void CloseFileLogging()
        {
            if (logWriter != null)
            {
                logWriter.Flush();
                logWriter.Close();
                logWriter = null;
                Debug.Log("[AerodynamicsController] File logging closed.");
            }
        }

        private void WriteLogHeader()
        {
            if (logWriter == null || logHeaderWritten) return;

            // Build header based on surfaces
            var header = "Time,PosX,PosY,PosZ,VelX,VelY,VelZ,VelMag,AngVelX,AngVelY,AngVelZ,AngVelMag,LocalAngVelX,LocalAngVelY,LocalAngVelZ,TotalForceX,TotalForceY,TotalForceZ,TotalForceMag,TotalTorqueX,TotalTorqueY,TotalTorqueZ,TotalTorqueMag,LocalTorqueX,LocalTorqueY,LocalTorqueZ,Thrust";

            foreach (var surface in aerodynamicSurfaces)
            {
                string name = surface.name.Replace(",", "_");
                header += $",{name}_ForceX,{name}_ForceY,{name}_ForceZ,{name}_ForceMag,{name}_TorqueX,{name}_TorqueY,{name}_TorqueZ,{name}_TorqueMag";
                header += $",{name}_PitchTorqueX,{name}_PitchTorqueY,{name}_PitchTorqueZ";
                header += $",{name}_RelPosX,{name}_RelPosY,{name}_RelPosZ,{name}_TorqueFromForceX,{name}_TorqueFromForceY,{name}_TorqueFromForceZ";
                header += $",{name}_LocalVelX,{name}_LocalVelY,{name}_LocalVelZ,{name}_AoADeg";
                header += $",{name}_DragDirX,{name}_DragDirY,{name}_DragDirZ";
                header += $",{name}_LiftDirX,{name}_LiftDirY,{name}_LiftDirZ";
                header += $",{name}_WorldFluidVelX,{name}_WorldFluidVelY,{name}_WorldFluidVelZ";
                header += $",{name}_ElementFluidVelX,{name}_ElementFluidVelY,{name}_ElementFluidVelZ";
                header += $",{name}_CurrentMedium,{name}_SubmersionRatio";
                // Transform directions for debugging coordinate system
                header += $",{name}_TransUpX,{name}_TransUpY,{name}_TransUpZ";
                header += $",{name}_TransRightX,{name}_TransRightY,{name}_TransRightZ";
                header += $",{name}_TransFwdX,{name}_TransFwdY,{name}_TransFwdZ";

                int elemCount = surface != null ? surface.Parameters.numElements : 0;
                for (int i = 0; i < elemCount; i++)
                {
                    header += $",{name}_Elem{i}_Cl,{name}_Elem{i}_Cd";
                }
            }

            logWriter.WriteLine(header);
            logHeaderWritten = true;
        }

        private void WriteLogEntry(List<SurfaceForce> surfaceForces, Vector3 velocity, Vector3 angularVelocity, BiVector3 totalForce, float thrust)
        {
            if (logWriter == null || !enableFileLogging) return;

            if (!logHeaderWritten)
            {
                WriteLogHeader();
            }

            Vector3 pos = GetPosition();
            Transform referenceTransform = GetReferenceTransform();
            Vector3 localAngularVelocity = referenceTransform != null
                ? referenceTransform.InverseTransformDirection(angularVelocity)
                : angularVelocity;
            Vector3 localTotalTorque = referenceTransform != null
                ? referenceTransform.InverseTransformDirection(totalForce.torque)
                : totalForce.torque;

            var line = $"{simulationTime:F4},{pos.x:F4},{pos.y:F4},{pos.z:F4},{velocity.x:F4},{velocity.y:F4},{velocity.z:F4},{velocity.magnitude:F4},{angularVelocity.x:F4},{angularVelocity.y:F4},{angularVelocity.z:F4},{angularVelocity.magnitude:F4},{localAngularVelocity.x:F4},{localAngularVelocity.y:F4},{localAngularVelocity.z:F4},{totalForce.force.x:F4},{totalForce.force.y:F4},{totalForce.force.z:F4},{totalForce.force.magnitude:F4},{totalForce.torque.x:F4},{totalForce.torque.y:F4},{totalForce.torque.z:F4},{totalForce.torque.magnitude:F4},{localTotalTorque.x:F4},{localTotalTorque.y:F4},{localTotalTorque.z:F4},{thrust:F4}";

            foreach (var sf in surfaceForces)
            {
                line += $",{sf.forces.force.x:F4},{sf.forces.force.y:F4},{sf.forces.force.z:F4},{sf.forces.force.magnitude:F4},{sf.forces.torque.x:F4},{sf.forces.torque.y:F4},{sf.forces.torque.z:F4},{sf.forces.torque.magnitude:F4}";
                Vector3 pitchTorque = sf.surface != null ? sf.surface.LastPitchTorque : Vector3.zero;
                line += $",{pitchTorque.x:F4},{pitchTorque.y:F4},{pitchTorque.z:F4}";
                line += $",{sf.relativePosition.x:F4},{sf.relativePosition.y:F4},{sf.relativePosition.z:F4},{sf.torqueFromForce.x:F4},{sf.torqueFromForce.y:F4},{sf.torqueFromForce.z:F4}";
                Vector3 localVel = sf.surface != null ? sf.surface.LastLocalVelocity : Vector3.zero;
                float aoaDeg = sf.surface != null ? sf.surface.LastAoADeg : 0f;
                Vector3 dragDir = sf.surface != null ? sf.surface.LastDragDirection : Vector3.zero;
                Vector3 liftDir = sf.surface != null ? sf.surface.LastLiftDirection : Vector3.zero;
                Vector3 worldFluidVel = sf.surface != null ? sf.surface.LastWorldFluidVelocity : Vector3.zero;
                Vector3 elementFluidVel = sf.surface != null ? sf.surface.LastElementFluidVelocity : Vector3.zero;
                line += $",{localVel.x:F4},{localVel.y:F4},{localVel.z:F4},{aoaDeg:F3}";
                line += $",{dragDir.x:F4},{dragDir.y:F4},{dragDir.z:F4}";
                line += $",{liftDir.x:F4},{liftDir.y:F4},{liftDir.z:F4}";
                line += $",{worldFluidVel.x:F4},{worldFluidVel.y:F4},{worldFluidVel.z:F4}";
                line += $",{elementFluidVel.x:F4},{elementFluidVel.y:F4},{elementFluidVel.z:F4}";
                int mediumValue = sf.surface != null ? (int)sf.surface.CurrentMedium : 0;
                float submersion = sf.surface != null ? sf.surface.SubmersionRatio : 0f;
                line += $",{mediumValue},{submersion:F4}";
                // Transform directions for debugging coordinate system
                Vector3 transUp = sf.surface != null ? sf.surface.transform.up : Vector3.up;
                Vector3 transRight = sf.surface != null ? sf.surface.transform.right : Vector3.right;
                Vector3 transFwd = sf.surface != null ? sf.surface.transform.forward : Vector3.forward;
                line += $",{transUp.x:F4},{transUp.y:F4},{transUp.z:F4}";
                line += $",{transRight.x:F4},{transRight.y:F4},{transRight.z:F4}";
                line += $",{transFwd.x:F4},{transFwd.y:F4},{transFwd.z:F4}";

                int elemCount = sf.surface != null ? sf.surface.Parameters.numElements : 0;
                var elems = sf.surface != null ? sf.surface.BladeElements : null;
                for (int i = 0; i < elemCount; i++)
                {
                    float cl = (elems != null && i < elems.Length) ? elems[i].currentCl : 0f;
                    float cd = (elems != null && i < elems.Length) ? elems[i].currentCd : 0f;
                    line += $",{cl:F4},{cd:F4}";
                }
            }

            logWriter.WriteLine(line);

            // Flush periodically to ensure data is written
            if (simulationTime % 1f < Time.fixedDeltaTime)
            {
                logWriter.Flush();
            }
        }

        private Vector3 GetPosition()
        {
            if (targetRigidbody != null)
                return targetRigidbody.position;
            if (targetArticulationBody != null)
                return targetArticulationBody.transform.position;
            return transform.position;
        }

        private Transform GetReferenceTransform()
        {
            if (targetRigidbody != null)
                return targetRigidbody.transform;
            if (targetArticulationBody != null)
                return targetArticulationBody.transform;
            return transform;
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
            PruneSurfaceVelocityCache();
            PruneSurfaceAngularVelocityCache();
            PruneSurfaceVerticalForceCache();

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

            simulationTime += Time.fixedDeltaTime;

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
                    velocity, angularVelocity, centerOfMass, true, out currentForceAndTorque);
            }

            // Apply forces per surface body
            ApplySurfaceForces(surfaceForces);

            // Apply thrust
            float currentThrust = 0f;
            if (thrustPercent > 0f)
            {
                Vector3 thrustForce = transform.TransformDirection(thrustDirection.normalized) *
                                     maxThrust * thrustPercent;
                ApplyForce(thrustForce);
                currentThrust = thrustForce.magnitude;
            }

            // Write log entry
            if (enableFileLogging)
            {
                WriteLogEntry(surfaceForces, velocity, angularVelocity, currentForceAndTorque, currentThrust);
            }
        }

        private struct SurfaceForce
        {
            public AeroSurface surface;
            public ArticulationBody articulationBody;
            public Rigidbody rigidbody;
            public BiVector3 forces;
            public Vector3 relativePosition;
            public Vector3 torqueFromForce;
        }

        private List<SurfaceForce> ComputeSurfaceForces(
            Vector3 defaultVelocity,
            Vector3 defaultAngularVelocity,
            Vector3 defaultCenterOfMass,
            bool updateFilterState,
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
                surfaceVelocity = GetFilteredSurfaceVelocity(surface, surfaceVelocity, updateFilterState);
                bodyAngularVelocity = GetFilteredAngularVelocity(surface, bodyAngularVelocity, updateFilterState);
                Vector3 relativePosition = surface.transform.position - defaultCenterOfMass;
                Vector3 surfacePointVelocity = surfaceVelocity;
                if (showDebugInfo)
                {
                    bool logSurface = surface.name.Contains("wing_left") || surface.name.Contains("wing_right");
                    if (logSurface)
                    {
                        Debug.Log("Name:" + surface.name + " surfaceVelocity: " + surfaceVelocity);
                        Debug.Log("Name:" + surface.name + " bodyAngularVelocity: " + bodyAngularVelocity);
                        Debug.Log("Name:" + surface.name + " bodyCenterOfMass: " + bodyCenterOfMass);
                        Debug.Log("Name:" + surface.name + " defaultCenterOfMass: " + defaultCenterOfMass);
                        Debug.Log("Name:" + surface.name + " relativePosition(defaultCOM): " + relativePosition);
                        Debug.Log("Name:" + surface.name + " surfacePointVelocity: " + surfacePointVelocity);
                    }
                }

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
                    Vector3 fluidVelocity = -surfacePointVelocity + ambientFluidVelocity;
                    // Pass angular velocity for per-element velocity calculation in BET
                    surfaceForces = surface.CalculateForcesAutoMedium(
                        fluidVelocity,
                        relativePosition,
                        bodyAngularVelocity);
                }
                else
                {
                    // Use manual density mode
                    // Determine ambient velocity based on configured medium
                    Vector3 ambientFluidVelocity = surface.Parameters.fluidMedium == FluidMedium.Water
                        ? underwaterCurrentVelocity
                        : windVelocity;

                    // Fluid velocity relative to surface
                    Vector3 fluidVelocity = -surfacePointVelocity + ambientFluidVelocity;
                    // Pass angular velocity for per-element velocity calculation in BET
                    surfaceForces = surface.CalculateForces(
                        fluidVelocity,
                        defaultFluidDensity,
                        relativePosition,
                        bodyAngularVelocity);
                }

                totalForceAndTorque += surfaceForces;
                Vector3 torqueFromForce = Vector3.Cross(relativePosition, surfaceForces.force);
                results.Add(new SurfaceForce
                {
                    surface = surface,
                    articulationBody = surfaceArticulation,
                    rigidbody = surfaceRigidbody,
                    forces = surfaceForces,
                    relativePosition = relativePosition,
                    torqueFromForce = torqueFromForce
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
                velocity, angularVelocity, centerOfMass, true, out currentTotal);

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
                predictedVelocity, angularVelocity, centerOfMass, false, out predictedTotal);

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
                return GetTotalArticulationCenterOfMass();
            return transform.position;
        }

        /// <summary>
        /// Calculates the total center of mass of all ArticulationBodies in the robot hierarchy
        /// </summary>
        private Vector3 GetTotalArticulationCenterOfMass()
        {
            if (allArticulationBodies == null || allArticulationBodies.Length == 0)
            {
                CacheArticulationBodies();
            }

            if (allArticulationBodies != null && allArticulationBodies.Length > 0)
            {
                float totalMass = 0f;
                Vector3 weightedSum = Vector3.zero;

                foreach (var body in allArticulationBodies)
                {
                    if (body == null) continue;
                    totalMass += body.mass;
                    weightedSum += body.worldCenterOfMass * body.mass;
                }

                if (totalMass > 0f)
                {
                    return weightedSum / totalMass;
                }
            }

            return targetArticulationBody != null
                ? targetArticulationBody.worldCenterOfMass
                : transform.position;
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

            for (int i = 0; i < surfaceForces.Count; i++)
            {
                SurfaceForce surfaceForce = surfaceForces[i];
                if (surfaceForce.surface != null)
                {
                    surfaceForce.forces.force = GetSmoothedVerticalForce(
                        surfaceForce.surface,
                        surfaceForce.forces.force);
                }

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

        private Vector3 GetFilteredSurfaceVelocity(
            AeroSurface surface,
            Vector3 rawVelocity,
            bool updateState)
        {
            if (!enableSurfaceVelocityFiltering || surface == null)
            {
                Vector3 clampedRaw = ClampSurfaceSpeed(rawVelocity);
                if (updateState && surface != null)
                {
                    filteredSurfaceVelocities[surface] = clampedRaw;
                }
                return clampedRaw;
            }

            Vector3 filteredVelocity = FilterSurfaceVelocity(surface, rawVelocity);
            Vector3 clampedVelocity = ClampSurfaceSpeed(filteredVelocity);

            if (updateState)
            {
                filteredSurfaceVelocities[surface] = clampedVelocity;
            }

            return clampedVelocity;
        }

        private Vector3 FilterSurfaceVelocity(AeroSurface surface, Vector3 rawVelocity)
        {
            // Check if filtering is disabled
            float dt = Time.fixedDeltaTime;
            if (surfaceVelocityFilterTime <= 0f || dt <= 0f)
            {
                return rawVelocity;
            }

            // Clamp the raw velocity first
            Vector3 clampedVelocity = ClampSurfaceSpeed(rawVelocity);

            // If no previous velocity exists, return the clamped velocity (no smoothing on first frame)
            if (!filteredSurfaceVelocities.TryGetValue(surface, out Vector3 previousVelocity))
            {
                return clampedVelocity;
            }

            // Apply low-pass filter (exponential smoothing)
            float alpha = dt / (surfaceVelocityFilterTime + dt);
            Vector3 blended = Vector3.Lerp(previousVelocity, clampedVelocity, alpha);

            return blended;
        }

        private Vector3 ClampSurfaceSpeed(Vector3 velocity)
        {
            if (maxSurfaceSpeed <= 0f)
            {
                return velocity;
            }

            return Vector3.ClampMagnitude(velocity, maxSurfaceSpeed);
        }

        private Vector3 GetFilteredAngularVelocity(
            AeroSurface surface,
            Vector3 rawAngularVelocity,
            bool updateState)
        {
            if (!enableAngularVelocityFiltering || surface == null)
            {
                Vector3 clampedRaw = ClampAngularSpeed(rawAngularVelocity);
                if (updateState && surface != null)
                {
                    filteredSurfaceAngularVelocities[surface] = clampedRaw;
                }
                return clampedRaw;
            }

            Vector3 filteredVelocity = FilterAngularVelocity(surface, rawAngularVelocity);
            Vector3 clampedVelocity = ClampAngularSpeed(filteredVelocity);

            if (updateState)
            {
                filteredSurfaceAngularVelocities[surface] = clampedVelocity;
            }

            return clampedVelocity;
        }

        private Vector3 FilterAngularVelocity(AeroSurface surface, Vector3 rawAngularVelocity)
        {
            float dt = Time.fixedDeltaTime;
            if (angularVelocityFilterTime <= 0f || dt <= 0f)
            {
                return rawAngularVelocity;
            }

            Vector3 clampedAngular = ClampAngularSpeed(rawAngularVelocity);

            if (!filteredSurfaceAngularVelocities.TryGetValue(surface, out Vector3 previousAngular))
            {
                return clampedAngular;
            }

            float alpha = dt / (angularVelocityFilterTime + dt);
            return Vector3.Lerp(previousAngular, clampedAngular, alpha);
        }

        private Vector3 ClampAngularSpeed(Vector3 angularVelocity)
        {
            if (maxAngularSpeed <= 0f)
            {
                return angularVelocity;
            }

            return Vector3.ClampMagnitude(angularVelocity, maxAngularSpeed);
        }

        private Vector3 GetSmoothedVerticalForce(AeroSurface surface, Vector3 rawForce)
        {
            Vector3 clampedForce = ClampUpwardForce(rawForce);
            if (!enableVerticalForceSmoothing || surface == null)
            {
                if (surface != null)
                {
                    filteredSurfaceVerticalForces[surface] = clampedForce.y;
                }
                return clampedForce;
            }

            float dt = Time.fixedDeltaTime;
            if (verticalForceFilterTime <= 0f || dt <= 0f)
            {
                filteredSurfaceVerticalForces[surface] = clampedForce.y;
                return clampedForce;
            }

            if (!filteredSurfaceVerticalForces.TryGetValue(surface, out float previousY))
            {
                filteredSurfaceVerticalForces[surface] = clampedForce.y;
                return clampedForce;
            }

            float alpha = dt / (verticalForceFilterTime + dt);
            float smoothedY = Mathf.Lerp(previousY, clampedForce.y, alpha);
            filteredSurfaceVerticalForces[surface] = smoothedY;
            return new Vector3(clampedForce.x, smoothedY, clampedForce.z);
        }

        private Vector3 ClampUpwardForce(Vector3 force)
        {
            if (maxUpwardForcePerSurface <= 0f)
            {
                return force;
            }

            if (force.y > maxUpwardForcePerSurface)
            {
                force.y = maxUpwardForcePerSurface;
            }

            return force;
        }

        private void PruneSurfaceVelocityCache()
        {
            if (filteredSurfaceVelocities.Count == 0 || aerodynamicSurfaces.Count == 0)
            {
                return;
            }

            for (int i = aerodynamicSurfaces.Count - 1; i >= 0; i--)
            {
                if (aerodynamicSurfaces[i] == null)
                {
                    aerodynamicSurfaces.RemoveAt(i);
                }
            }

            List<AeroSurface> staleSurfaces = null;
            foreach (var entry in filteredSurfaceVelocities)
            {
                if (!aerodynamicSurfaces.Contains(entry.Key))
                {
                    staleSurfaces ??= new List<AeroSurface>();
                    staleSurfaces.Add(entry.Key);
                }
            }

            if (staleSurfaces == null) return;

            foreach (var surface in staleSurfaces)
            {
                filteredSurfaceVelocities.Remove(surface);
            }
        }

        private void PruneSurfaceAngularVelocityCache()
        {
            if (filteredSurfaceAngularVelocities.Count == 0 || aerodynamicSurfaces.Count == 0)
            {
                return;
            }

            List<AeroSurface> staleSurfaces = null;
            foreach (var entry in filteredSurfaceAngularVelocities)
            {
                if (!aerodynamicSurfaces.Contains(entry.Key))
                {
                    staleSurfaces ??= new List<AeroSurface>();
                    staleSurfaces.Add(entry.Key);
                }
            }

            if (staleSurfaces == null) return;

            foreach (var surface in staleSurfaces)
            {
                filteredSurfaceAngularVelocities.Remove(surface);
            }
        }

        private void PruneSurfaceVerticalForceCache()
        {
            if (filteredSurfaceVerticalForces.Count == 0 || aerodynamicSurfaces.Count == 0)
            {
                return;
            }

            List<AeroSurface> staleSurfaces = null;
            foreach (var entry in filteredSurfaceVerticalForces)
            {
                if (!aerodynamicSurfaces.Contains(entry.Key))
                {
                    staleSurfaces ??= new List<AeroSurface>();
                    staleSurfaces.Add(entry.Key);
                }
            }

            if (staleSurfaces == null) return;

            foreach (var surface in staleSurfaces)
            {
                filteredSurfaceVerticalForces.Remove(surface);
            }
        }
    }
}
