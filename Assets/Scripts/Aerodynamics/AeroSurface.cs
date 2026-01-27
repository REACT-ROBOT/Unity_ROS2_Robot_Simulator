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
                elem.localPosition = new Vector3(-spanPos, 0f, 0f);

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
                Vector3 elementFluidVelocity = worldFluidVelocity - Vector3.Cross(angularVelocity, offset);

                // Transform world velocity to local space
                Vector3 localVelocity = transform.InverseTransformDirection(elementFluidVelocity);

                // Debug: Log velocity components before projection
                if (i == 0)
                {
                    Debug.Log($"[BET] {name} elem[0]: worldFluidVel={worldFluidVelocity}, elementFluidVel={elementFluidVelocity}");
                    Debug.Log($"[BET] {name} elem[0]: localVelocity(before)={localVelocity}");
                    Debug.Log($"[BET] {name} transform: forward={transform.forward}, up={transform.up}, right={transform.right}");
                }

                // Project to 2D (ignore sideslip)
                // Z = forward, Y = up in Unity local space, so zero X (span direction)
                localVelocity = new Vector3(0, localVelocity.y, localVelocity.z);

                if (i == 0)
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
                    if (i == 0)
                    {
                        Debug.Log($"[BET] {name} elem[0]: velocity too small, skipping");
                    }
                    continue;
                }

                // Calculate angle of attack for this element
                float localAoA = Mathf.Atan2(elem.currentVelocity.y, -elem.currentVelocity.z);
                localAoA += elem.localTwist;  // Add geometric twist
                elem.currentAoA = localAoA;

                // Estimate Cl for induced velocity calculation
                Vector3 coeffs = CalculateCoefficients(localAoA, correctedLiftSlope, zeroLiftAoA,
                    stallAngleHigh, stallAngleLow, medium);
                elem.currentCl = coeffs.x;
                elem.currentCd = coeffs.y;
                elem.currentCm = coeffs.z;

                if (i == 0)
                {
                    Debug.Log($"[BET] {name} elem[0]: AoA={localAoA * Mathf.Rad2Deg:F2}°, twist={elem.localTwist * Mathf.Rad2Deg:F2}°, Cl={coeffs.x:F4}, Cd={coeffs.y:F4}");
                }
            }

            // Compute induced velocities
            inducedVelocityModel.ComputeInducedVelocities(bladeElements, config, dt);

            // Step 3: Compute forces at each element with induced velocity correction
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
                Vector3 coeffs = CalculateCoefficients(effectiveAoA, localLiftSlope, zeroLiftAoA,
                    stallAngleHigh, stallAngleLow, medium);

                // Apply Wagner factor for unsteady effects
                float wagnerFactor = 1f;
                if (config.enableUnsteadyEffects && config.enableWagnerLag)
                {
                    wagnerFactor = unsteadyEffects.ComputeWagnerFactor(elem, dt);
                    coeffs.x *= wagnerFactor;  // Reduce lift during transients
                }

                elem.currentCl = coeffs.x;
                elem.currentCd = coeffs.y;
                elem.currentCm = coeffs.z;

                // Dynamic pressure for this element
                float q = 0.5f * density * effectiveVelocity.sqrMagnitude;

                // Force directions in world space
                // Use element's actual fluid velocity direction (accounts for angular velocity)
                Vector3 worldPos = transform.TransformPoint(elem.localPosition);
                Vector3 offset = worldPos - transform.position;
                Vector3 elementFluidVelocity = worldFluidVelocity - Vector3.Cross(angularVelocity, offset);
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
                // Lift is perpendicular to both drag direction and span axis
                // Span axis is local X (transform.right in world space)
                // Cross(right, drag) gives lift direction by right-hand rule
                Vector3 liftCross = Vector3.Cross(transform.right, dragDirection);
                Vector3 liftDirection;
                if (liftCross.sqrMagnitude < 0.0001f)
                {
                    // Fallback if drag and span axis are nearly parallel
                    liftDirection = transform.up;
                }
                else
                {
                    liftDirection = liftCross.normalized;
                }

                // Forces for this element
                Vector3 lift = liftDirection * coeffs.x * q * elem.elementArea;
                Vector3 drag = dragDirection * coeffs.y * q * elem.elementArea;

                if (i == 0)
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

                // Add unsteady torque
                totalForces.torque += unsteadyForces.torque;

                // Update history for next frame
                elem.UpdateHistory();
            }

            Debug.Log($"[BET] {name} TOTAL: force={totalForces.force} (mag={totalForces.force.magnitude:E3}N), torque={totalForces.torque}");
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
            if (waterAdapter == null)
            {
                currentMedium = CurrentMedium.Air;
                submersionRatio = 0f;
                return;
            }

            float waterHeight = waterAdapter.GetWaterLevel(transform.position);
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
            Debug.Log("worldFluidVelocity(before): " + worldFluidVelocity);
            Debug.Log("localVelocity(before): " + localVelocity);

            // Project to 2D (ignore sideslip for simplified model)
            // Z = forward, Y = up in local space
            localVelocity = new Vector3(0, localVelocity.y, localVelocity.z);

            if (localVelocity.sqrMagnitude < 0.0001f)
            {
                lastCalculatedForces = forceAndTorque;
                return forceAndTorque;
            }

            // Calculate Reynolds number for the current conditions
            float velocity = localVelocity.magnitude;
            lastReynoldsNumber = config.CalculateReynoldsNumber(velocity, medium);

            // Calculate angle of attack
            float angleOfAttack = Mathf.Atan2(localVelocity.y, -localVelocity.z);

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

            // Dynamic pressure: q = 0.5 * ρ * V²
            float dynamicPressure = 0.5f * density * localVelocity.sqrMagnitude;
            float area = config.Area;

            // Calculate force directions in world space using local flow direction and span axis
            Vector3 localDragDirection = localVelocity.normalized;
            Vector3 dragDirection = transform.TransformDirection(localDragDirection);

            Vector3 localLiftDirection = Vector3.Cross(localDragDirection, Vector3.right).normalized;
            if (localLiftDirection.sqrMagnitude < 0.01f)
            {
                localLiftDirection = Vector3.up;
            }
            Debug.Log("localLiftDirection: " + localLiftDirection);

            Vector3 liftDirection = transform.TransformDirection(localLiftDirection);
            Debug.Log("liftDirection: " + liftDirection);

            // Apply forces
            Vector3 lift = liftDirection * coefficients.x * dynamicPressure * area;
            Vector3 drag = dragDirection * coefficients.y * dynamicPressure * area;

            forceAndTorque.force = lift + drag;

            // Calculate torque from moment coefficient
            Vector3 pitchTorque = -transform.forward * coefficients.z * dynamicPressure * area * config.chord;
            forceAndTorque.torque = pitchTorque;

            // Add torque from force acting at offset from center of mass
            forceAndTorque.torque += Vector3.Cross(relativePosition, forceAndTorque.force);

            lastCalculatedForces = forceAndTorque;
            return forceAndTorque;
        }

        /// <summary>
        /// Calculates lift, drag, and moment coefficients with stall model
        /// </summary>
        private Vector3 CalculateCoefficients(
            float angleOfAttack,
            float correctedLiftSlope,
            float zeroLiftAoA,
            float stallAngleHigh,
            float stallAngleLow,
            CurrentMedium medium)
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
                    angleOfAttack, correctedLiftSlope, zeroLiftAoA, medium);
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
                    angleOfAttack, correctedLiftSlope, zeroLiftAoA, medium);

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
        private Vector3 CalculateCoefficientsAtLowAoA(
            float angleOfAttack,
            float correctedLiftSlope,
            float zeroLiftAoA,
            CurrentMedium medium)
        {
            var config = Parameters;
            float ar = config.EffectiveAspectRatio;
            float skinFriction = config.GetSkinFriction(medium);

            // Linear lift coefficient
            float liftCoefficient = correctedLiftSlope * (angleOfAttack - zeroLiftAoA);

            // Induced angle from lift (Prandtl's formula)
            float inducedAngle = ar > 0.01f ? liftCoefficient / (Mathf.PI * ar) : 0f;
            float effectiveAngle = angleOfAttack - zeroLiftAoA - inducedAngle;

            // Tangential (friction) and normal force coefficients
            float tangentialCoeff = skinFriction * Mathf.Cos(effectiveAngle);

            float normalCoeff = (liftCoefficient +
                Mathf.Sin(effectiveAngle) * tangentialCoeff) /
                Mathf.Cos(effectiveAngle);

            // Total drag: form drag + induced drag + friction
            float dragCoefficient = normalCoeff * Mathf.Sin(effectiveAngle) +
                                   tangentialCoeff * Mathf.Cos(effectiveAngle);

            // Moment coefficient (quarter-chord approximation)
            float torqueCoefficient = -normalCoeff * TorqueCoefficientProportion(effectiveAngle);

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

            float torqueCoefficient = -normalCoeff * TorqueCoefficientProportion(effectiveAngle);

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
