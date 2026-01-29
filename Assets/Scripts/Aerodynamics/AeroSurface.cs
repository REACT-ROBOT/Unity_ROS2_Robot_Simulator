// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Aerodynamic surface physics calculations
// Based on Aircraft-Physics by JoeBrow (MIT License) and Khan & Nahon 2015
// Extended for underwater compatibility with automatic medium detection
// Extended with Blade Element Theory for improved accuracy on discretized wings

using UnityEngine;
using Hydrodynamics;
using Aerodynamics.BladeElement;

namespace Aerodynamics
{
    /// <summary>
    /// Control surface input type for control mapping
    /// </summary>
    public enum ControlInputType
    {
        None,
        Pitch,
        Roll,
        Yaw,
        Flap
    }

    /// <summary>
    /// Aerodynamic surface component that calculates lift, drag, and moment forces.
    /// Implements a full stall model with smooth transitions.
    /// Supports both air and underwater environments with automatic medium detection.
    /// </summary>
    public class AeroSurface : MonoBehaviour
    {
        [Header("Configuration")]
        [SerializeField]
        private bool useConfigAsset = false;

        [SerializeField]
        private AeroSurfaceConfig configAsset;

        [SerializeField]
        private AeroSurfaceParameters inlineParameters = new AeroSurfaceParameters();

        [SerializeField]
        [Tooltip("Invert span direction for mirrored wings if roll damping has wrong sign")]
        private bool invertSpanDirection = false;

        [Header("Control Surface Settings")]
        [SerializeField]
        private bool isControlSurface = false;

        [SerializeField]
        private ControlInputType inputType = ControlInputType.None;

        [SerializeField]
        [Tooltip("Multiplier for control input (-1 for inverted)")]
        private float inputMultiplier = 1f;

        [Header("Debug")]
        [SerializeField]
        private bool showDebugForces = false;

        [SerializeField]
        private float debugForceScale = 0.01f;

        [Header("Moment Limiting")]
        [SerializeField]
        [Tooltip("Clamp AoA used for pitch moment coefficient (deg). Set <= 0 to disable.")]
        private float maxMomentAoADeg = 90f;

        [SerializeField]
        [Tooltip("Fade pitch moment beyond stall. 1 keeps moment at stall, 0 fades to zero by 90 deg.")]
        private float postStallMomentScale = 0.2f;

        [Header("Post-Stall Force Fade")]
        [SerializeField]
        [Tooltip("Fade lift/drag beyond stall. 1 keeps force at stall, 0 fades to zero by 90 deg.")]
        private float postStallForceScale = 0.2f;

        [Header("Coefficient Limiting")]
        [SerializeField]
        [Tooltip("Maximum absolute lift coefficient (typical airfoil: 1.5-2.0)")]
        private float maxClMagnitude = 2.5f;

        [SerializeField]
        [Tooltip("Maximum absolute drag coefficient (flat plate at 90deg: ~2.0)")]
        private float maxCdMagnitude = 2.5f;

        [Header("Coefficient Rate Limiting (Anti-Oscillation)")]
        [SerializeField]
        [Tooltip("Maximum Cl change rate per second. 0 = no limit. Helps prevent pitch oscillation.")]
        private float maxClRatePerSecond = 5.0f;

        [SerializeField]
        [Tooltip("Maximum Cd change rate per second. 0 = no limit.")]
        private float maxCdRatePerSecond = 5.0f;

        // For debug visualization
        //[SerializeField]
        //private bool showMediumState = false;

        // Runtime state
        private float flapAngle = 0f;  // Current flap deflection in radians
        private CurrentMedium currentMedium = CurrentMedium.Air;
        private float submersionRatio = 0f;  // 0 = in air, 1 = fully submerged

        // Cached calculations
        private BiVector3 lastCalculatedForces;
        private float lastReynoldsNumber;
        private Vector3 lastLocalVelocity;
        private float lastAoADeg;
        private Vector3 lastPitchTorque;
        private Vector3 lastDragDirection;
        private Vector3 lastLiftDirection;
        private Vector3 lastWorldFluidVelocity;
        private Vector3 lastElementFluidVelocity;

        // Rate limiting state (for QS mode - single element)
        private float prevClQS = 0f;
        private float prevCdQS = 0f;
        private bool prevCoeffsInitializedQS = false;

        // Blade Element Theory components
        private BladeElementState[] bladeElements;
        private IInducedVelocityModel inducedVelocityModel;
        private UnsteadyEffects unsteadyEffects;
        private bool betInitialized = false;

        /// <summary>
        /// Gets the active parameters (from asset or inline)
        /// </summary>
        public AeroSurfaceParameters Parameters => useConfigAsset && configAsset != null
            ? configAsset.parameters
            : inlineParameters;

        /// <summary>
        /// Whether this surface acts as a control surface
        /// </summary>
        public bool IsControlSurface => isControlSurface;

        /// <summary>
        /// The control input type for this surface
        /// </summary>
        public ControlInputType InputType => inputType;

        /// <summary>
        /// Input multiplier for control mapping
        /// </summary>
        public float InputMultiplier => inputMultiplier;

        /// <summary>
        /// Gets the currently detected medium
        /// </summary>
        public CurrentMedium CurrentMedium => currentMedium;

        /// <summary>
        /// Gets the submersion ratio (0 = air, 1 = fully submerged)
        /// </summary>
        public float SubmersionRatio => submersionRatio;

        /// <summary>
        /// Gets the last calculated Reynolds number
        /// </summary>
        public float LastReynoldsNumber => lastReynoldsNumber;

        /// <summary>
        /// Gets the last local velocity used for AoA (projected to Y/Z)
        /// </summary>
        public Vector3 LastLocalVelocity => lastLocalVelocity;

        /// <summary>
        /// Gets the last angle of attack in degrees
        /// </summary>
        public float LastAoADeg => lastAoADeg;

        /// <summary>
        /// Gets the last drag direction in world space
        /// </summary>
        public Vector3 LastDragDirection => lastDragDirection;
        public Vector3 LastLiftDirection => lastLiftDirection;

        /// <summary>
        /// Gets the last world fluid velocity used for force calculation
        /// </summary>
        public Vector3 LastWorldFluidVelocity => lastWorldFluidVelocity;

        /// <summary>
        /// Gets the last element fluid velocity (element 0) in world space
        /// </summary>
        public Vector3 LastElementFluidVelocity => lastElementFluidVelocity;

        /// <summary>
        /// Gets the last pitch torque (moment coefficient contribution) in world space
        /// </summary>
        public Vector3 LastPitchTorque => lastPitchTorque;

        /// <summary>
        /// Sets the flap/control surface deflection angle
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        public void SetFlapAngle(float angle)
        {
            float maxAngleRad = Parameters.maxFlapAngle * Mathf.Deg2Rad;
            flapAngle = Mathf.Clamp(angle, -maxAngleRad, maxAngleRad);
        }

        /// <summary>
        /// Gets the current flap angle in radians
        /// </summary>
        public float GetFlapAngle() => flapAngle;

        /// <summary>
        /// Gets the blade element states (for debugging/visualization)
        /// </summary>
        public BladeElementState[] BladeElements => bladeElements;

        /// <summary>
        /// Whether Blade Element Theory is active (numElements > 1)
        /// </summary>
        public bool UsesBladeElementTheory => Parameters.numElements > 1;

        /// <summary>
        /// Initialize blade elements for BET calculation
        /// </summary>
        private void InitializeBladeElements()
        {
            var config = Parameters;
            int n = config.numElements;

            if (n <= 1)
            {
                bladeElements = null;
                betInitialized = false;
                return;
            }

            bladeElements = new BladeElementState[n];
            float elementSpan = config.span / n;

            for (int i = 0; i < n; i++)
            {
                bladeElements[i] = new BladeElementState();
                var elem = bladeElements[i];

                // Spanwise fraction (0 to 1 along the wing)
                elem.spanwiseFraction = (i + 0.5f) / n;

                // Local position along span (Y-axis in local space typically)(ROS: Y-axis -> Unity: -X-axis)
                // Centered at wing midpoint: goes from -span/2 to +span/2
                float spanPos = (elem.spanwiseFraction - 0.5f) * config.span;
                float spanSign = invertSpanDirection ? -1f : 1f;
                elem.localPosition = new Vector3(-spanPos * spanSign, 0f, 0f);

                // Local chord with taper
                // taper_ratio = tip_chord / root_chord
                // chord(y) = root_chord * (1 - (1 - taper) * |2y/span|)
                float normalizedSpan = Mathf.Abs(2f * (elem.spanwiseFraction - 0.5f));
                elem.localChord = config.chord * (1f - (1f - config.taperRatio) * normalizedSpan);

                // Linear twist distribution
                float twistFraction = Mathf.Abs(2f * (elem.spanwiseFraction - 0.5f));
                elem.localTwist = Mathf.Lerp(config.twistRoot, config.twistTip, twistFraction) * Mathf.Deg2Rad;

                // Element dimensions
                elem.elementSpan = elementSpan;
                elem.elementArea = elem.localChord * elementSpan;
            }

            // Initialize induced velocity model
            inducedVelocityModel = InducedVelocityModelFactory.Create(config.inducedVelocityModel);
            inducedVelocityModel.Initialize(config);

            // Initialize unsteady effects
            unsteadyEffects = new UnsteadyEffects();
            unsteadyEffects.Initialize(config);

            betInitialized = true;
        }

        /// <summary>
        /// Calculate forces using Blade Element Theory
        /// </summary>
        /// <param name="worldFluidVelocity">Fluid velocity at surface center in world space</param>
        /// <param name="density">Fluid density in kg/m³</param>
        /// <param name="relativePosition">Position relative to center of mass for torque calculation</param>
        /// <param name="medium">Current fluid medium</param>
        /// <param name="angularVelocity">Body angular velocity in world space (for per-element velocity)</param>
        private BiVector3 CalculateForcesWithBET(Vector3 worldFluidVelocity, float density,
            Vector3 relativePosition, CurrentMedium medium, Vector3 angularVelocity = default)
        {
            var config = Parameters;
            float dt = Time.fixedDeltaTime;

            // Ensure elements are initialized
            if (!betInitialized || bladeElements == null || bladeElements.Length != config.numElements)
            {
                InitializeBladeElements();
            }

            BiVector3 totalForces = BiVector3.zero;

            // Pre-compute shared values
            float ar = config.EffectiveAspectRatio;
            float correctedLiftSlope = config.liftSlope * ar / (ar + 2f * (ar + 4f) / (ar + 2f));
            var stallAngles = config.GetStallAngles(medium);
            float zeroLiftAoaBase = config.zeroLiftAoA * Mathf.Deg2Rad;
            float stallAngleHighBase = stallAngles.high * Mathf.Deg2Rad;
            float stallAngleLowBase = stallAngles.low * Mathf.Deg2Rad;

            // Flap effects (shared across elements for simplicity)
            float theta = Mathf.Acos(2f * config.flapFraction - 1f);
            float flapEffectiveness = 1f - (theta - Mathf.Sin(theta)) / Mathf.PI;
            float deltaLift = correctedLiftSlope * flapEffectiveness *
                              FlapEffectivenessCorrection(flapAngle) * flapAngle;
            float zeroLiftAoA = zeroLiftAoaBase - deltaLift / correctedLiftSlope;

            float clMaxHigh = correctedLiftSlope * (stallAngleHighBase - zeroLiftAoaBase) +
                              deltaLift * LiftCoefficientMaxFraction(config.flapFraction);
            float clMaxLow = correctedLiftSlope * (stallAngleLowBase - zeroLiftAoaBase) +
                             deltaLift * LiftCoefficientMaxFraction(config.flapFraction);
            float stallAngleHigh = zeroLiftAoA + clMaxHigh / correctedLiftSlope;
            float stallAngleLow = zeroLiftAoA + clMaxLow / correctedLiftSlope;

            // Step 1: Compute velocity at each element
            for (int i = 0; i < bladeElements.Length; i++)
            {
                var elem = bladeElements[i];

                // World position of this element
                Vector3 worldPos = transform.TransformPoint(elem.localPosition);

                // Calculate fluid velocity at this element's position
                // If body is rotating, elements at different positions have different velocities
                // v_element = v_center - ω × (element_pos - surface_center)
                Vector3 offset = worldPos - transform.position;
                Vector3 elementFluidVelocity = worldFluidVelocity;// + Vector3.Cross(angularVelocity, offset);

                // Transform world velocity to local space
                Vector3 localVelocity = transform.InverseTransformDirection(elementFluidVelocity);

                // Debug: Log velocity components before projection
                if (i == 0 && showDebugForces)
                {
                    Debug.Log($"[BET] {name} elem[0]: worldFluidVel={worldFluidVelocity}, elementFluidVel={elementFluidVelocity}");
                    Debug.Log($"[BET] {name} elem[0]: localVelocity(before)={localVelocity}");
                    Debug.Log($"[BET] {name} transform: forward={transform.forward}, up={transform.up}, right={transform.right}");
                }

                if (i == 0 && showDebugForces)
                {
                    Debug.Log($"[BET] {name} elem[0]: localVelocity(after)={localVelocity}");
                }

                elem.currentVelocity = localVelocity;
            }

            // Step 2: Compute induced velocities (first pass for Cl estimation)
            for (int i = 0; i < bladeElements.Length; i++)
            {
                var elem = bladeElements[i];
                if (elem.currentVelocity.sqrMagnitude < 0.0001f)
                {
                    if (i == 0 && showDebugForces)
                    {
                        Debug.Log($"[BET] {name} elem[0]: velocity too small, skipping");
                    }
                    continue;
                }

                // Calculate angle of attack for this element
                float localAoA = Mathf.Atan2(elem.currentVelocity.y, -elem.currentVelocity.z);
                localAoA += elem.localTwist;  // Add geometric twist
                elem.currentAoA = localAoA;
                if (i == 0)
                {
                    lastLocalVelocity = elem.currentVelocity;
                    lastAoADeg = localAoA * Mathf.Rad2Deg;
                }

                // Estimate Cl for induced velocity calculation
                // Use skipInducedDrag=true because induced effects are handled by the induced velocity model
                Vector3 coeffs = CalculateCoefficients(localAoA, correctedLiftSlope, zeroLiftAoA,
                    stallAngleHigh, stallAngleLow, medium, skipInducedDrag: true);
                float forceScale = GetPostStallForceScale(localAoA, stallAngleHigh, stallAngleLow);
                elem.currentCl = coeffs.x * forceScale;
                elem.currentCd = coeffs.y * forceScale;
                elem.currentCm = coeffs.z;

                if (i == 0 && showDebugForces)
                {
                    Debug.Log($"[BET] {name} elem[0]: AoA={localAoA * Mathf.Rad2Deg:F2}°, twist={elem.localTwist * Mathf.Rad2Deg:F2}°, Cl={coeffs.x:F4}, Cd={coeffs.y:F4}");
                }
            }

            // Compute induced velocities
            inducedVelocityModel.ComputeInducedVelocities(bladeElements, config, dt);

            // Step 3: Compute forces at each element with induced velocity correction
            Vector3 totalPitchTorque = Vector3.zero;
            for (int i = 0; i < bladeElements.Length; i++)
            {
                var elem = bladeElements[i];

                if (elem.currentVelocity.sqrMagnitude < 0.0001f)
                {
                    elem.UpdateHistory();
                    continue;
                }

                // Add induced velocity to get effective velocity
                Vector3 effectiveVelocity = elem.currentVelocity + elem.inducedVelocity;

                // Recalculate AoA with induced velocity
                // Z = forward in Unity local space
                float effectiveAoA = Mathf.Atan2(effectiveVelocity.y, -effectiveVelocity.z);
                effectiveAoA += elem.localTwist;

                // Apply tip-loss factor to lift slope
                float localLiftSlope = correctedLiftSlope * elem.tipLossFactor;

                // Calculate coefficients
                // Use skipInducedDrag=true because induced effects are handled by the induced velocity model
                Vector3 coeffs = CalculateCoefficients(effectiveAoA, localLiftSlope, zeroLiftAoA,
                    stallAngleHigh, stallAngleLow, medium, skipInducedDrag: true);

                // Apply Wagner factor for unsteady effects
                float wagnerFactor = 1f;
                if (config.enableUnsteadyEffects && config.enableWagnerLag)
                {
                    wagnerFactor = unsteadyEffects.ComputeWagnerFactor(elem, dt);
                    coeffs.x *= wagnerFactor;  // Reduce lift during transients
                }

                float forceScale = GetPostStallForceScale(effectiveAoA, stallAngleHigh, stallAngleLow);
                elem.currentCl = coeffs.x * forceScale;
                elem.currentCd = coeffs.y * forceScale;
                elem.currentCm = coeffs.z;

                // Clamp coefficients to physically reasonable values
                // This prevents numerical instability from extreme AoA or rapid changes
                elem.currentCl = Mathf.Clamp(elem.currentCl, -maxClMagnitude, maxClMagnitude);
                elem.currentCd = Mathf.Clamp(elem.currentCd, 0f, maxCdMagnitude);

                // Apply coefficient rate limiting to prevent oscillation
                if (elem.prevCoeffsInitialized && maxClRatePerSecond > 0f)
                {
                    float maxClChange = maxClRatePerSecond * dt;
                    elem.currentCl = Mathf.Clamp(elem.currentCl,
                        elem.previousCl - maxClChange,
                        elem.previousCl + maxClChange);
                }
                if (elem.prevCoeffsInitialized && maxCdRatePerSecond > 0f)
                {
                    float maxCdChange = maxCdRatePerSecond * dt;
                    elem.currentCd = Mathf.Clamp(elem.currentCd,
                        elem.previousCd - maxCdChange,
                        elem.previousCd + maxCdChange);
                }

                // Dynamic pressure for this element
                float q = 0.5f * density * effectiveVelocity.sqrMagnitude;

                // Force directions in world space
                // Use element's actual fluid velocity direction (accounts for angular velocity)
                Vector3 worldPos = transform.TransformPoint(elem.localPosition);
                Vector3 offset = worldPos - transform.position;
                Vector3 elementFluidVelocity = worldFluidVelocity;// + Vector3.Cross(angularVelocity, offset);
                Vector3 dragDirection;
                if (elementFluidVelocity.sqrMagnitude < 0.0001f)
                {
                    // Fallback to center velocity if element velocity is too small
                    dragDirection = worldFluidVelocity.sqrMagnitude > 0.0001f
                        ? worldFluidVelocity.normalized
                        : Vector3.forward;
                }
                else
                {
                    dragDirection = elementFluidVelocity.normalized;
                }
                if (i == 0)
                {
                    lastDragDirection = dragDirection;
                }
                // Lift direction calculation with stability improvements:
                // 1. Lift is perpendicular to flow direction and lies in the wing plane
                // 2. The reference direction is the wing's up vector (transform.up)
                // 3. Cl sign determines whether lift is toward suction or pressure side
                //
                // Method: Project drag direction onto wing plane, then find perpendicular
                Vector3 wingNormal = transform.up;
                Vector3 spanAxis = invertSpanDirection ? -transform.right : transform.right;

                // Project flow direction onto wing plane (remove component along wing normal)
                float dragDotNormal = Vector3.Dot(dragDirection, wingNormal);
                Vector3 flowOnPlane = dragDirection - dragDotNormal * wingNormal;

                Vector3 liftDirection;
                if (flowOnPlane.sqrMagnitude < 0.0001f)
                {
                    // Flow is nearly perpendicular to wing (stall/flat plate regime)
                    // Use forward direction as reference for lift
                    liftDirection = transform.forward;
                }
                else
                {
                    // Lift is perpendicular to flow-on-plane and span axis
                    // Cross product gives direction in wing plane, perpendicular to flow
                    Vector3 liftCross = Vector3.Cross(flowOnPlane.normalized, spanAxis);

                    if (liftCross.sqrMagnitude < 0.0001f)
                    {
                        // Flow is parallel to span (sideslip case)
                        liftDirection = wingNormal;
                    }
                    else
                    {
                        liftDirection = liftCross.normalized;
                        // Ensure lift direction has positive component along wing normal
                        // This makes positive Cl produce lift toward suction side (wing top)
                        if (Vector3.Dot(liftDirection, wingNormal) < 0f)
                        {
                            liftDirection = -liftDirection;
                        }
                    }
                }

                if (i == 0)
                {
                    lastLiftDirection = liftDirection;
                }

                // Forces for this element
                Vector3 lift = liftDirection * elem.currentCl * q * elem.elementArea;
                Vector3 drag = dragDirection * elem.currentCd * q * elem.elementArea;

                if (i == 0 && showDebugForces)
                {
                    Debug.Log($"[BET] {name} elem[0]: spanAxis(right)={transform.right}, effectiveAoA={effectiveAoA * Mathf.Rad2Deg:F2}°");
                    Debug.Log($"[BET] {name} elem[0]: q={q:F2}, area={elem.elementArea:F6}, Cl={coeffs.x:F4}");
                    Debug.Log($"[BET] {name} elem[0]: liftDir={liftDirection}, dragDir={dragDirection}");
                    float liftMag = coeffs.x * q * elem.elementArea;
                    float dragMag = coeffs.y * q * elem.elementArea;
                    Debug.Log($"[BET] {name} elem[0]: liftMag={liftMag:E3}N, dragMag={dragMag:E3}N, lift={lift}, drag={drag}");
                }

                elem.lastLiftForce = lift;
                elem.lastDragForce = drag;

                // Add unsteady forces
                BiVector3 unsteadyForces = BiVector3.zero;
                if (config.enableUnsteadyEffects)
                {
                    unsteadyForces = unsteadyEffects.ComputeUnsteadyForces(elem, density, dt);
                }

                // Torque calculation uses worldPos already computed above
                Vector3 elementRelativePos = worldPos - (transform.position - relativePosition);

                Vector3 elementForce = lift + drag + unsteadyForces.force;

                totalForces.force += elementForce;

                // Torque from force at offset position
                totalForces.torque += Vector3.Cross(elementRelativePos, elementForce);

                // Pitch moment
                Vector3 pitchTorque = -transform.forward * coeffs.z * q * elem.elementArea * elem.localChord;
                totalForces.torque += pitchTorque;
                totalPitchTorque += pitchTorque;

                // Add unsteady torque
                totalForces.torque += unsteadyForces.torque;

                // Update history for next frame
                elem.UpdateHistory();
            }

            if (showDebugForces)
            {
                Debug.Log($"[BET] {name} TOTAL: force={totalForces.force} (mag={totalForces.force.magnitude:E3}N), torque={totalForces.torque}");
            }
            lastPitchTorque = totalPitchTorque;
            return totalForces;
        }

        /// <summary>
        /// Detects the current medium based on water surface position
        /// </summary>
        private void DetectMedium()
        {
            var config = Parameters;

            // If not in auto mode, use configured medium
            if (config.fluidMedium != FluidMedium.Auto)
            {
                currentMedium = config.fluidMedium == FluidMedium.Water
                    ? CurrentMedium.Water
                    : CurrentMedium.Air;
                submersionRatio = currentMedium == CurrentMedium.Water ? 1f : 0f;
                return;
            }

            // Try to get water height from NaughtyWaterHeightAdapter
            var waterAdapter = NaughtyWaterHeightAdapter.Instance;

            // Check if water adapter exists AND has a valid water volume
            if (waterAdapter == null || !waterAdapter.HasValidWaterVolume)
            {
                currentMedium = CurrentMedium.Air;
                submersionRatio = 0f;
                return;
            }

            float waterHeight = waterAdapter.GetWaterLevel(transform.position);

            // If GetWaterLevel returns negative infinity, there's no valid water
            if (float.IsNegativeInfinity(waterHeight))
            {
                currentMedium = CurrentMedium.Air;
                submersionRatio = 0f;
                return;
            }

            float surfaceHeight = transform.position.y;

            // Calculate submersion based on surface geometry
            float halfSpan = config.span * 0.5f;
            float topOfSurface = surfaceHeight + halfSpan;
            float bottomOfSurface = surfaceHeight - halfSpan;

            if (bottomOfSurface >= waterHeight)
            {
                // Fully in air
                currentMedium = CurrentMedium.Air;
                submersionRatio = 0f;
            }
            else if (topOfSurface <= waterHeight)
            {
                // Fully submerged
                currentMedium = CurrentMedium.Water;
                submersionRatio = 1f;
            }
            else
            {
                // Partially submerged (transition)
                currentMedium = CurrentMedium.Transition;
                submersionRatio = (waterHeight - bottomOfSurface) / config.span;
                submersionRatio = Mathf.Clamp01(submersionRatio);
            }
        }

        /// <summary>
        /// Calculates aerodynamic forces with automatic medium detection
        /// </summary>
        /// <param name="worldFluidVelocity">Fluid velocity relative to the surface in world space</param>
        /// <param name="relativePosition">Position relative to center of mass for torque calculation</param>
        /// <param name="angularVelocity">Body angular velocity in world space (for BET per-element velocity)</param>
        /// <returns>Combined force and torque</returns>
        public BiVector3 CalculateForcesAutoMedium(Vector3 worldFluidVelocity, Vector3 relativePosition,
            Vector3 angularVelocity = default)
        {
            DetectMedium();

            var config = Parameters;
            float density;

            if (currentMedium == CurrentMedium.Transition)
            {
                // Interpolate density based on submersion
                density = Mathf.Lerp(config.airDensity, config.waterDensity, submersionRatio);
            }
            else
            {
                density = config.GetDensity(currentMedium);
            }

            return CalculateForcesWithMedium(worldFluidVelocity, density, relativePosition, currentMedium, angularVelocity);
        }

        /// <summary>
        /// Calculates aerodynamic forces and torques from air/fluid velocity
        /// </summary>
        /// <param name="worldFluidVelocity">Fluid velocity relative to the surface in world space</param>
        /// <param name="fluidDensity">Fluid density (kg/m³). If 0, uses auto-detection</param>
        /// <param name="relativePosition">Position relative to center of mass for torque calculation</param>
        /// <param name="angularVelocity">Body angular velocity in world space (for BET per-element velocity)</param>
        /// <returns>Combined force and torque</returns>
        public BiVector3 CalculateForces(Vector3 worldFluidVelocity, float fluidDensity, Vector3 relativePosition,
            Vector3 angularVelocity = default)
        {
            // If density is not provided, use auto-detection
            if (fluidDensity <= 0)
            {
                return CalculateForcesAutoMedium(worldFluidVelocity, relativePosition, angularVelocity);
            }

            // Determine medium based on provided density
            var config = Parameters;
            CurrentMedium medium;
            if (Mathf.Abs(fluidDensity - config.waterDensity) < Mathf.Abs(fluidDensity - config.airDensity))
            {
                medium = CurrentMedium.Water;
            }
            else
            {
                medium = CurrentMedium.Air;
            }

            return CalculateForcesWithMedium(worldFluidVelocity, fluidDensity, relativePosition, medium, angularVelocity);
        }

        /// <summary>
        /// Calculates aerodynamic forces with explicit medium specification
        /// </summary>
        /// <param name="worldFluidVelocity">Fluid velocity at surface center in world space</param>
        /// <param name="fluidDensity">Fluid density in kg/m³</param>
        /// <param name="relativePosition">Position relative to center of mass for torque calculation</param>
        /// <param name="medium">Current fluid medium</param>
        /// <param name="angularVelocity">Body angular velocity in world space (for BET per-element velocity)</param>
        private BiVector3 CalculateForcesWithMedium(Vector3 worldFluidVelocity, float fluidDensity,
            Vector3 relativePosition, CurrentMedium medium, Vector3 angularVelocity = default)
        {
            var config = Parameters;

            // Use provided density
            float density = fluidDensity;
            // Always record the current fluid velocity for logging
            lastWorldFluidVelocity = worldFluidVelocity;
            lastElementFluidVelocity = worldFluidVelocity;

            // Use Blade Element Theory if numElements > 1
            if (config.numElements > 1)
            {
                BiVector3 betForces = CalculateForcesWithBET(worldFluidVelocity, density, relativePosition, medium, angularVelocity);
                lastCalculatedForces = betForces;
                return betForces;
            }

            // Original quasi-steady calculation for single element (backward compatible)
            BiVector3 forceAndTorque = BiVector3.zero;

            // Transform velocity to local space
            Vector3 localVelocity = transform.InverseTransformDirection(worldFluidVelocity);
            lastWorldFluidVelocity = worldFluidVelocity;
            lastElementFluidVelocity = worldFluidVelocity;
            if (showDebugForces)
            {
                Debug.Log($"[QS] {name}: worldFluidVelocity={worldFluidVelocity}");
                Debug.Log($"[QS] {name}: localVelocity(before)={localVelocity}");
            }

            // Project to 2D (ignore sideslip for simplified model)
            // Z = forward, Y = up in local space
            localVelocity = new Vector3(0, localVelocity.y, localVelocity.z);

            if (localVelocity.sqrMagnitude < 0.0001f)
            {
                lastPitchTorque = Vector3.zero;
                lastCalculatedForces = forceAndTorque;
                return forceAndTorque;
            }

            // Calculate Reynolds number for the current conditions
            float velocity = localVelocity.magnitude;
            lastReynoldsNumber = config.CalculateReynoldsNumber(velocity, medium);

            // Calculate angle of attack
            float angleOfAttack = Mathf.Atan2(localVelocity.y, -localVelocity.z);
            lastLocalVelocity = localVelocity;
            lastAoADeg = angleOfAttack * Mathf.Rad2Deg;
            if (showDebugForces)
            {
                Debug.Log($"[QS] {name}: localVelocity(after)={localVelocity}, AoA={angleOfAttack * Mathf.Rad2Deg:F2}°");
            }

            // Apply aspect ratio correction to lift slope (finite wing effect)
            float ar = config.EffectiveAspectRatio;
            float correctedLiftSlope = config.liftSlope * ar /
                (ar + 2f * (ar + 4f) / (ar + 2f));

            // Get medium-specific stall angles
            var stallAngles = config.GetStallAngles(medium);

            // Calculate flap effects on aerodynamic characteristics
            float zeroLiftAoaBase = config.zeroLiftAoA * Mathf.Deg2Rad;
            float stallAngleHighBase = stallAngles.high * Mathf.Deg2Rad;
            float stallAngleLowBase = stallAngles.low * Mathf.Deg2Rad;

            // Flap effectiveness (geometric relationship)
            float theta = Mathf.Acos(2f * config.flapFraction - 1f);
            float flapEffectiveness = 1f - (theta - Mathf.Sin(theta)) / Mathf.PI;

            // Apply flap deflection effect with nonlinear correction
            float deltaLift = correctedLiftSlope * flapEffectiveness *
                              FlapEffectivenessCorrection(flapAngle) * flapAngle;

            // Modified aerodynamic reference points due to flap deflection
            float zeroLiftAoA = zeroLiftAoaBase - deltaLift / correctedLiftSlope;

            float clMaxHigh = correctedLiftSlope * (stallAngleHighBase - zeroLiftAoaBase) +
                              deltaLift * LiftCoefficientMaxFraction(config.flapFraction);
            float clMaxLow = correctedLiftSlope * (stallAngleLowBase - zeroLiftAoaBase) +
                             deltaLift * LiftCoefficientMaxFraction(config.flapFraction);

            float stallAngleHigh = zeroLiftAoA + clMaxHigh / correctedLiftSlope;
            float stallAngleLow = zeroLiftAoA + clMaxLow / correctedLiftSlope;

            // Calculate aerodynamic coefficients with stall model
            Vector3 coefficients = CalculateCoefficients(
                angleOfAttack,
                correctedLiftSlope,
                zeroLiftAoA,
                stallAngleHigh,
                stallAngleLow,
                medium);
            float forceScale = GetPostStallForceScale(angleOfAttack, stallAngleHigh, stallAngleLow);
            coefficients.x *= forceScale;
            coefficients.y *= forceScale;

            // Clamp coefficients to physically reasonable values
            coefficients.x = Mathf.Clamp(coefficients.x, -maxClMagnitude, maxClMagnitude);
            coefficients.y = Mathf.Clamp(coefficients.y, 0f, maxCdMagnitude);

            // Apply coefficient rate limiting to prevent oscillation (QS mode)
            float dt = Time.fixedDeltaTime;
            if (prevCoeffsInitializedQS && maxClRatePerSecond > 0f)
            {
                float maxClChange = maxClRatePerSecond * dt;
                coefficients.x = Mathf.Clamp(coefficients.x,
                    prevClQS - maxClChange,
                    prevClQS + maxClChange);
            }
            if (prevCoeffsInitializedQS && maxCdRatePerSecond > 0f)
            {
                float maxCdChange = maxCdRatePerSecond * dt;
                coefficients.y = Mathf.Clamp(coefficients.y,
                    prevCdQS - maxCdChange,
                    prevCdQS + maxCdChange);
            }
            // Update previous coefficients for next frame
            prevClQS = coefficients.x;
            prevCdQS = coefficients.y;
            prevCoeffsInitializedQS = true;

            // Dynamic pressure: q = 0.5 * ρ * V²
            float dynamicPressure = 0.5f * density * localVelocity.sqrMagnitude;
            float area = config.Area;

            // Calculate force directions in world space using local flow direction and span axis
            Vector3 localDragDirection = localVelocity.normalized;
            Vector3 dragDirection = transform.TransformDirection(localDragDirection);
            lastDragDirection = dragDirection;

            // Lift direction calculation (local space) with stability improvements:
            // Project flow onto wing plane, then find perpendicular direction
            Vector3 spanLocal = invertSpanDirection ? Vector3.left : Vector3.right;

            // In local space, wing normal is Vector3.up
            float dragDotUp = localDragDirection.y;
            Vector3 flowOnPlane = localDragDirection - dragDotUp * Vector3.up;

            Vector3 localLiftDirection;
            if (flowOnPlane.sqrMagnitude < 0.01f)
            {
                // Flow is nearly perpendicular to wing (stall/flat plate)
                localLiftDirection = Vector3.forward;
            }
            else
            {
                // Lift perpendicular to flow-on-plane
                Vector3 liftCross = Vector3.Cross(flowOnPlane.normalized, spanLocal);
                if (liftCross.sqrMagnitude < 0.01f)
                {
                    localLiftDirection = Vector3.up;
                }
                else
                {
                    localLiftDirection = liftCross.normalized;
                    // Ensure lift has positive Y component (toward wing top in local space)
                    if (localLiftDirection.y < 0f)
                    {
                        localLiftDirection = -localLiftDirection;
                    }
                }
            }
            if (showDebugForces)
            {
                Debug.Log("[QS] " + name + ": localLiftDirection=" + localLiftDirection);
            }

            Vector3 liftDirection = transform.TransformDirection(localLiftDirection);
            lastLiftDirection = liftDirection;
            if (showDebugForces)
            {
                Debug.Log("[QS] " + name + ": liftDirection=" + liftDirection);
            }

            // Apply forces
            Vector3 lift = liftDirection * coefficients.x * dynamicPressure * area;
            Vector3 drag = dragDirection * coefficients.y * dynamicPressure * area;

            forceAndTorque.force = lift + drag;

            // Calculate torque from moment coefficient
            Vector3 pitchTorque = -transform.forward * coefficients.z * dynamicPressure * area * config.chord;
            forceAndTorque.torque = pitchTorque;
            lastPitchTorque = pitchTorque;

            // Add torque from force acting at offset from center of mass
            forceAndTorque.torque += Vector3.Cross(relativePosition, forceAndTorque.force);

            lastCalculatedForces = forceAndTorque;
            return forceAndTorque;
        }

        /// <summary>
        /// Calculates lift, drag, and moment coefficients with stall model
        /// </summary>
        /// <param name="skipInducedDrag">If true, skip induced drag calculation (for BET mode)</param>
        private Vector3 CalculateCoefficients(
            float angleOfAttack,
            float correctedLiftSlope,
            float zeroLiftAoA,
            float stallAngleHigh,
            float stallAngleLow,
            CurrentMedium medium,
            bool skipInducedDrag = false)
        {
            Vector3 coefficients;

            // Define padding for smooth stall transition
            float paddingAngleHigh = Mathf.Deg2Rad * Mathf.Lerp(15f, 5f,
                (Mathf.Rad2Deg * Mathf.Abs(flapAngle) + 50f) / 100f);
            float paddingAngleLow = Mathf.Deg2Rad * Mathf.Lerp(15f, 5f,
                (-Mathf.Rad2Deg * Mathf.Abs(flapAngle) + 50f) / 100f);

            float paddedStallAngleHigh = stallAngleHigh + paddingAngleHigh;
            float paddedStallAngleLow = stallAngleLow - paddingAngleLow;

            if (angleOfAttack < stallAngleHigh && angleOfAttack > stallAngleLow)
            {
                // Normal flight regime (linear lift region)
                coefficients = CalculateCoefficientsAtLowAoA(
                    angleOfAttack, correctedLiftSlope, zeroLiftAoA, medium, skipInducedDrag);
            }
            else if (angleOfAttack > paddedStallAngleHigh || angleOfAttack < paddedStallAngleLow)
            {
                // Full stall regime
                coefficients = CalculateCoefficientsAtStall(
                    angleOfAttack, correctedLiftSlope, zeroLiftAoA,
                    stallAngleHigh, stallAngleLow, medium);
            }
            else
            {
                // Transition region - interpolate between normal and stall
                Vector3 lowAoACoeffs = CalculateCoefficientsAtLowAoA(
                    angleOfAttack, correctedLiftSlope, zeroLiftAoA, medium, skipInducedDrag);

                Vector3 stallCoeffs = CalculateCoefficientsAtStall(
                    angleOfAttack, correctedLiftSlope, zeroLiftAoA,
                    stallAngleHigh, stallAngleLow, medium);

                float lerpParam;
                if (angleOfAttack > stallAngleHigh)
                {
                    lerpParam = (angleOfAttack - stallAngleHigh) /
                                (paddedStallAngleHigh - stallAngleHigh);
                }
                else
                {
                    lerpParam = (stallAngleLow - angleOfAttack) /
                                (stallAngleLow - paddedStallAngleLow);
                }

                coefficients = Vector3.Lerp(lowAoACoeffs, stallCoeffs, lerpParam);
            }

            return coefficients;
        }

        /// <summary>
        /// Calculates coefficients in normal flight regime (linear lift)
        /// </summary>
        /// <param name="skipInducedDrag">If true, skip induced drag calculation (for BET mode where induced effects are handled by induced velocity model)</param>
        private Vector3 CalculateCoefficientsAtLowAoA(
            float angleOfAttack,
            float correctedLiftSlope,
            float zeroLiftAoA,
            CurrentMedium medium,
            bool skipInducedDrag = false)
        {
            var config = Parameters;
            float ar = config.EffectiveAspectRatio;
            float skinFriction = config.GetSkinFriction(medium);

            // Linear lift coefficient
            float liftCoefficient = correctedLiftSlope * (angleOfAttack - zeroLiftAoA);

            float dragCoefficient;
            float torqueCoefficient;

            if (skipInducedDrag)
            {
                // BET mode: induced drag is handled by induced velocity model
                // Only calculate parasitic drag (skin friction + form drag)
                float absAoA = Mathf.Abs(angleOfAttack - zeroLiftAoA);

                // Parasitic drag: Cd0 + k*Cl^2 (form drag from pressure distribution)
                // For thin airfoils, form drag is roughly proportional to AoA^2
                float formDragCoeff = 0.01f * absAoA * absAoA;  // Small form drag contribution

                // Total parasitic drag = skin friction + form drag
                dragCoefficient = skinFriction + formDragCoeff;

                // Simple moment coefficient
                torqueCoefficient = -liftCoefficient * 0.25f;  // Quarter-chord approximation
            }
            else
            {
                // Legacy mode: include induced drag via Prandtl's formula
                // Induced angle from lift (Prandtl's formula)
                float inducedAngle = ar > 0.01f ? liftCoefficient / (Mathf.PI * ar) : 0f;
                float effectiveAngle = angleOfAttack - zeroLiftAoA - inducedAngle;

                // Tangential (friction) and normal force coefficients
                float tangentialCoeff = skinFriction * Mathf.Cos(effectiveAngle);

                float normalCoeff = (liftCoefficient +
                    Mathf.Sin(effectiveAngle) * tangentialCoeff) /
                    Mathf.Cos(effectiveAngle);

                // Total drag: form drag + induced drag + friction
                dragCoefficient = normalCoeff * Mathf.Sin(effectiveAngle) +
                                  tangentialCoeff * Mathf.Cos(effectiveAngle);

                // Moment coefficient (quarter-chord approximation)
                float clampedMomentAngle = ClampMomentAoA(effectiveAngle);
                torqueCoefficient = -normalCoeff * TorqueCoefficientProportion(clampedMomentAngle);
            }

            return new Vector3(liftCoefficient, dragCoefficient, torqueCoefficient);
        }

        /// <summary>
        /// Calculates coefficients in stall regime (separated flow)
        /// </summary>
        private Vector3 CalculateCoefficientsAtStall(
            float angleOfAttack,
            float correctedLiftSlope,
            float zeroLiftAoA,
            float stallAngleHigh,
            float stallAngleLow,
            CurrentMedium medium)
        {
            var config = Parameters;
            float ar = config.EffectiveAspectRatio;
            float skinFriction = config.GetSkinFriction(medium);

            // Use stall angle lift as base
            float liftCoeffAtStall;
            if (angleOfAttack > stallAngleHigh)
            {
                liftCoeffAtStall = correctedLiftSlope * (stallAngleHigh - zeroLiftAoA);
            }
            else
            {
                liftCoeffAtStall = correctedLiftSlope * (stallAngleLow - zeroLiftAoA);
            }

            // Induced angle decay in stall
            float inducedAngle = ar > 0.01f ? liftCoeffAtStall / (Mathf.PI * ar) : 0f;

            // Gradually reduce induced angle as AoA increases beyond stall
            float stallAngle = angleOfAttack > 0 ? stallAngleHigh : stallAngleLow;
            float lerpParam = (Mathf.PI / 2f - Mathf.Clamp(Mathf.Abs(angleOfAttack), 0, Mathf.PI / 2f)) /
                              (Mathf.PI / 2f - Mathf.Abs(stallAngle));
            lerpParam = Mathf.Clamp01(lerpParam);
            inducedAngle = Mathf.Lerp(0f, inducedAngle, lerpParam);

            float effectiveAngle = angleOfAttack - zeroLiftAoA - inducedAngle;

            // Post-stall normal force coefficient (flat plate + AR correction)
            float normalCoeff = FrictionAt90Degrees(flapAngle) * Mathf.Sin(effectiveAngle) *
                (1f / (0.56f + 0.44f * Mathf.Abs(Mathf.Sin(effectiveAngle))) -
                0.41f * (1f - Mathf.Exp(-17f / Mathf.Max(ar, 0.1f))));

            float tangentialCoeff = 0.5f * skinFriction * Mathf.Cos(effectiveAngle);

            // Decompose to lift and drag
            float liftCoefficient = normalCoeff * Mathf.Cos(effectiveAngle) -
                                   tangentialCoeff * Mathf.Sin(effectiveAngle);
            float dragCoefficient = normalCoeff * Mathf.Sin(effectiveAngle) +
                                   tangentialCoeff * Mathf.Cos(effectiveAngle);

            float clampedMomentAngle = ClampMomentAoA(effectiveAngle);
            float momentScale = GetPostStallMomentScale(angleOfAttack, stallAngleHigh, stallAngleLow);
            float torqueCoefficient = -normalCoeff * TorqueCoefficientProportion(clampedMomentAngle) * momentScale;

            return new Vector3(liftCoefficient, dragCoefficient, torqueCoefficient);
        }

        /// <summary>
        /// Torque coefficient proportion based on effective angle
        /// Range: ~0.075 to ~0.25
        /// </summary>
        private float TorqueCoefficientProportion(float effectiveAngle)
        {
            return 0.25f - 0.175f * (1f - 2f * Mathf.Abs(effectiveAngle) / Mathf.PI);
        }

        private float ClampMomentAoA(float angleRad)
        {
            if (maxMomentAoADeg <= 0f)
            {
                return angleRad;
            }

            float limit = maxMomentAoADeg * Mathf.Deg2Rad;
            return Mathf.Clamp(angleRad, -limit, limit);
        }

        private float GetPostStallMomentScale(float angleOfAttack, float stallAngleHigh, float stallAngleLow)
        {
            float absAoA = Mathf.Abs(angleOfAttack);
            float absStall = Mathf.Abs(angleOfAttack >= 0f ? stallAngleHigh : stallAngleLow);
            float start = Mathf.Max(absStall, 0f);
            float end = Mathf.Max(start, Mathf.PI * 0.5f);

            if (absAoA <= start)
            {
                return 1f;
            }

            float t = (absAoA - start) / Mathf.Max(end - start, 1e-4f);
            t = Mathf.Clamp01(t);
            return Mathf.Lerp(1f, postStallMomentScale, t);
        }

        private float GetPostStallForceScale(float angleOfAttack, float stallAngleHigh, float stallAngleLow)
        {
            float absAoA = Mathf.Abs(angleOfAttack);
            float absStall = Mathf.Abs(angleOfAttack >= 0f ? stallAngleHigh : stallAngleLow);
            float start = Mathf.Max(absStall, 0f);
            float end = Mathf.Max(start, Mathf.PI * 0.5f);

            if (absAoA <= start)
            {
                return 1f;
            }

            float t = (absAoA - start) / Mathf.Max(end - start, 1e-4f);
            t = Mathf.Clamp01(t);
            return Mathf.Lerp(1f, postStallForceScale, t);
        }

        /// <summary>
        /// Flap effectiveness correction for large deflections
        /// Returns 0.8 at small angles, 0.4 at ±50°
        /// </summary>
        private float FlapEffectivenessCorrection(float flapAngle)
        {
            return Mathf.Lerp(0.8f, 0.4f, (Mathf.Abs(flapAngle) * Mathf.Rad2Deg - 10f) / 50f);
        }

        /// <summary>
        /// Maximum lift coefficient fraction based on flap size
        /// </summary>
        private float LiftCoefficientMaxFraction(float flapFraction)
        {
            return Mathf.Clamp01(1f - 0.5f * (flapFraction - 0.1f) / 0.3f);
        }

        /// <summary>
        /// Friction coefficient at 90° AoA (flat plate), affected by flap deflection
        /// </summary>
        private float FrictionAt90Degrees(float flapAngle)
        {
            return 1.98f - 4.26e-2f * flapAngle * flapAngle + 2.1e-1f * flapAngle;
        }

        private void OnDrawGizmosSelected()
        {
            if (!showDebugForces) return;

            var config = Parameters;

            // Draw surface outline
            Gizmos.color = Color.cyan;
            Vector3 halfChord = transform.right * config.chord * 0.5f;
            Vector3 halfSpan = transform.forward * config.span * 0.5f;

            Vector3[] corners = new Vector3[]
            {
                transform.position - halfChord - halfSpan,
                transform.position + halfChord - halfSpan,
                transform.position + halfChord + halfSpan,
                transform.position - halfChord + halfSpan
            };

            for (int i = 0; i < 4; i++)
            {
                Gizmos.DrawLine(corners[i], corners[(i + 1) % 4]);
            }

            // Draw forces if available
            if (lastCalculatedForces.force.sqrMagnitude > 0.001f)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(transform.position,
                    transform.position + lastCalculatedForces.force * debugForceScale);

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(transform.position,
                    transform.position + lastCalculatedForces.torque * debugForceScale);
            }
        }
    }
}
