// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Blade Element state for discretized aerodynamic surface calculations
// Supports fixed-wing, rotors, and flapping wings

using UnityEngine;

namespace Aerodynamics.BladeElement
{
    /// <summary>
    /// Represents the state of a single blade element along the span.
    /// Stores geometry, current state, and history for unsteady effects.
    /// </summary>
    [System.Serializable]
    public class BladeElementState
    {
        // Geometry (set during initialization, constant during simulation)

        /// <summary>
        /// Position of element center in local coordinates (relative to surface origin)
        /// </summary>
        public Vector3 localPosition;

        /// <summary>
        /// Spanwise fraction: 0 = root, 1 = tip (for single wing)
        /// For symmetric wings: -1 to 1 with 0 at center
        /// </summary>
        public float spanwiseFraction;

        /// <summary>
        /// Local chord length at this element (accounts for taper)
        /// </summary>
        public float localChord;

        /// <summary>
        /// Local geometric twist angle in radians (positive = nose up)
        /// </summary>
        public float localTwist;

        /// <summary>
        /// Element span length (total span / numElements)
        /// </summary>
        public float elementSpan;

        /// <summary>
        /// Element area = localChord * elementSpan
        /// </summary>
        public float elementArea;

        // Current state (updated each frame)

        /// <summary>
        /// Current velocity at element position in world coordinates
        /// Includes body velocity + angular velocity contribution
        /// </summary>
        public Vector3 currentVelocity;

        /// <summary>
        /// Current angle of attack in radians (geometric + induced)
        /// </summary>
        public float currentAoA;

        /// <summary>
        /// Current lift coefficient
        /// </summary>
        public float currentCl;

        /// <summary>
        /// Current drag coefficient
        /// </summary>
        public float currentCd;

        /// <summary>
        /// Current moment coefficient
        /// </summary>
        public float currentCm;

        // Induced velocity (from induced velocity model)

        /// <summary>
        /// Induced velocity at this element (from wake/vortex effects)
        /// </summary>
        public Vector3 inducedVelocity;

        /// <summary>
        /// Prandtl tip-loss factor (1 = no loss, 0 = full loss)
        /// </summary>
        public float tipLossFactor = 1f;

        // State history (for unsteady effects)

        /// <summary>
        /// Velocity from previous timestep (for acceleration calculation)
        /// </summary>
        public Vector3 previousVelocity;

        /// <summary>
        /// Angle of attack from previous timestep
        /// </summary>
        public float previousAoA;

        /// <summary>
        /// Circulation state for Wagner function lag
        /// </summary>
        public float circulationLag;

        /// <summary>
        /// Quasi-steady circulation (target for lag model)
        /// </summary>
        public float circulationQuasiSteady;

        /// <summary>
        /// Cumulative semi-chords traveled (for Wagner function)
        /// </summary>
        public float semiChordsTraveled;

        // Computed forces (for debugging/visualization)

        /// <summary>
        /// Last computed lift force at this element
        /// </summary>
        public Vector3 lastLiftForce;

        /// <summary>
        /// Last computed drag force at this element
        /// </summary>
        public Vector3 lastDragForce;

        /// <summary>
        /// Last computed added mass force
        /// </summary>
        public Vector3 lastAddedMassForce;

        /// <summary>
        /// Creates a new blade element state with default values
        /// </summary>
        public BladeElementState()
        {
            localPosition = Vector3.zero;
            spanwiseFraction = 0f;
            localChord = 1f;
            localTwist = 0f;
            elementSpan = 1f;
            elementArea = 1f;

            currentVelocity = Vector3.zero;
            currentAoA = 0f;
            currentCl = 0f;
            currentCd = 0f;
            currentCm = 0f;

            inducedVelocity = Vector3.zero;
            tipLossFactor = 1f;

            previousVelocity = Vector3.zero;
            previousAoA = 0f;
            circulationLag = 0f;
            circulationQuasiSteady = 0f;
            semiChordsTraveled = 0f;

            lastLiftForce = Vector3.zero;
            lastDragForce = Vector3.zero;
            lastAddedMassForce = Vector3.zero;
        }

        /// <summary>
        /// Resets the state history (call when simulation resets)
        /// </summary>
        public void ResetHistory()
        {
            previousVelocity = currentVelocity;
            previousAoA = currentAoA;
            circulationLag = 0f;
            circulationQuasiSteady = 0f;
            semiChordsTraveled = 0f;
            inducedVelocity = Vector3.zero;
        }

        /// <summary>
        /// Updates history after force calculation (call at end of each timestep)
        /// </summary>
        public void UpdateHistory()
        {
            previousVelocity = currentVelocity;
            previousAoA = currentAoA;
        }

        /// <summary>
        /// Computes the dynamic pressure at this element
        /// </summary>
        /// <param name="density">Fluid density in kg/m³</param>
        /// <returns>Dynamic pressure q = 0.5 * rho * V²</returns>
        public float GetDynamicPressure(float density)
        {
            return 0.5f * density * currentVelocity.sqrMagnitude;
        }

        /// <summary>
        /// Computes acceleration from velocity change
        /// </summary>
        /// <param name="dt">Timestep in seconds</param>
        /// <returns>Acceleration vector in m/s²</returns>
        public Vector3 GetAcceleration(float dt)
        {
            if (dt <= 0f) return Vector3.zero;
            return (currentVelocity - previousVelocity) / dt;
        }

        /// <summary>
        /// Computes rate of change of angle of attack
        /// </summary>
        /// <param name="dt">Timestep in seconds</param>
        /// <returns>AoA rate in rad/s</returns>
        public float GetAoARate(float dt)
        {
            if (dt <= 0f) return 0f;
            return (currentAoA - previousAoA) / dt;
        }
    }
}
