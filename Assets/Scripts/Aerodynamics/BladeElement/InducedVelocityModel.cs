// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Induced velocity models for blade element theory
// Supports fixed-wing (Prandtl), rotors (momentum), and simple wake models

using UnityEngine;

namespace Aerodynamics.BladeElement
{
    /// <summary>
    /// Interface for induced velocity calculation models
    /// </summary>
    public interface IInducedVelocityModel
    {
        /// <summary>
        /// Initialize the model with surface parameters
        /// </summary>
        void Initialize(AeroSurfaceParameters parameters);

        /// <summary>
        /// Compute induced velocities for all blade elements
        /// </summary>
        /// <param name="elements">Array of blade element states</param>
        /// <param name="parameters">Surface parameters</param>
        /// <param name="dt">Time step in seconds</param>
        void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt);
    }

    /// <summary>
    /// Factory for creating induced velocity models
    /// </summary>
    public static class InducedVelocityModelFactory
    {
        public static IInducedVelocityModel Create(InducedVelocityModelType type)
        {
            return type switch
            {
                InducedVelocityModelType.None => new NoInducedVelocity(),
                InducedVelocityModelType.Prandtl => new PrandtlInducedVelocity(),
                InducedVelocityModelType.Momentum => new MomentumInducedVelocity(),
                InducedVelocityModelType.SimpleWake => new SimpleWakeInducedVelocity(),
                _ => new PrandtlInducedVelocity()
            };
        }
    }

    /// <summary>
    /// No induced velocity correction (baseline for comparison)
    /// </summary>
    public class NoInducedVelocity : IInducedVelocityModel
    {
        public void Initialize(AeroSurfaceParameters parameters) { }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            // No induced velocity - each element uses freestream only
            foreach (var elem in elements)
            {
                elem.inducedVelocity = Vector3.zero;
                elem.tipLossFactor = 1f;
            }
        }
    }

    /// <summary>
    /// Prandtl tip-loss correction for finite wings
    /// Best for fixed-wing aircraft
    /// Based on Prandtl's lifting-line theory
    /// </summary>
    public class PrandtlInducedVelocity : IInducedVelocityModel
    {
        private float aspectRatio;

        public void Initialize(AeroSurfaceParameters parameters)
        {
            aspectRatio = parameters.EffectiveAspectRatio;
        }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            if (elements == null || elements.Length == 0) return;

            float ar = parameters.EffectiveAspectRatio;
            float span = parameters.span;

            for (int i = 0; i < elements.Length; i++)
            {
                BladeElementState elem = elements[i];

                // Distance from wing center (normalized: 0 at center, 1 at tip)
                // For single wing: spanwiseFraction goes 0 to 1
                // We need distance from center: |2*f - 1| for symmetric, or f for half-wing
                float r_R = Mathf.Abs(2f * elem.spanwiseFraction - 1f);

                // Get inflow angle from current velocity
                // Unity local space: Z = forward, Y = up
                float Vz = -elem.currentVelocity.z;  // Forward velocity (negative because flow comes from front)
                float Vy = elem.currentVelocity.y;   // Vertical velocity component
                float phi = Mathf.Atan2(Vy, Mathf.Max(Vz, 0.1f));

                // Prandtl tip-loss factor: F = (2/π) * acos(exp(-f))
                // where f = (B/2) * (1 - r/R) / (r/R * |sin(φ)|)
                // B = number of blades (use AR as proxy for wing)
                float sinPhi = Mathf.Abs(Mathf.Sin(phi));
                sinPhi = Mathf.Max(sinPhi, 0.01f);  // Avoid division by zero

                float f_tip = (ar / 2f) * (1f - r_R) / (Mathf.Max(r_R, 0.01f) * sinPhi);
                float F = (2f / Mathf.PI) * Mathf.Acos(Mathf.Clamp(Mathf.Exp(-f_tip), -1f, 1f));
                F = Mathf.Clamp(F, 0.01f, 1f);

                elem.tipLossFactor = F;

                // Induced velocity from lifting-line theory
                // w_i = Γ / (2πy) integrated, simplified as:
                // w_i ≈ Cl * V * c / (4 * span * F)
                float V = elem.currentVelocity.magnitude;
                float Cl = elem.currentCl;
                float c = elem.localChord;

                if (V > 0.1f && span > 0.01f && F > 0.01f)
                {
                    float w_i = Cl * V * c / (4f * span * F);
                    // Induced velocity is downward (negative Y in typical orientation)
                    elem.inducedVelocity = new Vector3(0f, -w_i, 0f);
                }
                else
                {
                    elem.inducedVelocity = Vector3.zero;
                }
            }
        }
    }

    /// <summary>
    /// Momentum theory induced velocity for rotors/propellers
    /// Uses actuator disk theory with blade element corrections
    /// </summary>
    public class MomentumInducedVelocity : IInducedVelocityModel
    {
        private const int MAX_ITERATIONS = 10;
        private const float CONVERGENCE_TOLERANCE = 0.001f;

        public void Initialize(AeroSurfaceParameters parameters) { }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            if (elements == null || elements.Length == 0) return;

            float span = parameters.span;
            float density = parameters.airDensity;  // TODO: support water

            // For rotor: use simple momentum theory
            // Thrust T = 2 * rho * A * v_i * (V + v_i) for climb
            // In hover: T = 2 * rho * A * v_i^2, so v_i = sqrt(T / (2 * rho * A))

            // Sum thrust from all elements to get total, then distribute induced velocity
            float totalThrust = 0f;
            foreach (var elem in elements)
            {
                // Approximate thrust from lift coefficient
                float q = elem.GetDynamicPressure(density);
                float thrust = elem.currentCl * q * elem.elementArea;
                totalThrust += Mathf.Abs(thrust);
            }

            // Disk area (assuming circular rotor, approximate)
            float diskArea = Mathf.PI * span * span / 4f;
            if (diskArea < 0.01f) diskArea = 0.01f;

            // Hover induced velocity
            float v_i_hover = Mathf.Sqrt(totalThrust / (2f * density * diskArea));

            // Apply to each element with radial variation
            for (int i = 0; i < elements.Length; i++)
            {
                BladeElementState elem = elements[i];

                // Radial position factor (induced velocity varies along blade)
                float r_R = elem.spanwiseFraction;

                // Prandtl tip-loss
                float f_tip = Mathf.Max(1f - r_R, 0.01f);
                float F = (2f / Mathf.PI) * Mathf.Acos(Mathf.Clamp(Mathf.Exp(-2f / f_tip), -1f, 1f));
                F = Mathf.Clamp(F, 0.01f, 1f);
                elem.tipLossFactor = F;

                // Local induced velocity (higher at tip for uniform loading)
                float v_i_local = v_i_hover * F;

                // Induced velocity is perpendicular to disk plane (Unity: Y-up)
                elem.inducedVelocity = new Vector3(0f, -v_i_local, 0f);
            }
        }
    }

    /// <summary>
    /// Simple wake model for flapping wings
    /// Uses quasi-steady approximation with time-lagged wake effects
    /// </summary>
    public class SimpleWakeInducedVelocity : IInducedVelocityModel
    {
        private Vector3[] previousInducedVelocities;
        private const float WAKE_DECAY_RATE = 0.9f;  // How quickly wake effects decay

        public void Initialize(AeroSurfaceParameters parameters) { }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            if (elements == null || elements.Length == 0) return;

            // Initialize previous velocities if needed
            if (previousInducedVelocities == null || previousInducedVelocities.Length != elements.Length)
            {
                previousInducedVelocities = new Vector3[elements.Length];
            }

            float span = parameters.span;

            for (int i = 0; i < elements.Length; i++)
            {
                BladeElementState elem = elements[i];

                // Tip loss factor (simple linear)
                float r_R = Mathf.Abs(2f * elem.spanwiseFraction - 1f);
                elem.tipLossFactor = 1f - 0.3f * r_R * r_R;  // Quadratic tip loss

                // Quasi-steady induced velocity
                float V = elem.currentVelocity.magnitude;
                float Cl = elem.currentCl;
                float c = elem.localChord;

                float w_i_qs = 0f;
                if (V > 0.1f && span > 0.01f)
                {
                    w_i_qs = Cl * V * c / (4f * span);
                }

                Vector3 w_i_new = new Vector3(0f, -w_i_qs, 0f);

                // Apply wake decay (low-pass filter for smooth transitions)
                // This models the lag in wake development during unsteady motion
                float alpha = 1f - Mathf.Exp(-dt / 0.1f);  // Time constant ~0.1s
                elem.inducedVelocity = Vector3.Lerp(previousInducedVelocities[i], w_i_new, alpha);

                // Add wake from previous motion (self-induced effects)
                // This creates a simple "memory" of previous circulation
                elem.inducedVelocity += previousInducedVelocities[i] * WAKE_DECAY_RATE * dt;

                // Store for next frame
                previousInducedVelocities[i] = elem.inducedVelocity;

                // Clamp to reasonable values
                float maxInduced = V * 0.5f;  // Induced velocity shouldn't exceed half freestream
                if (elem.inducedVelocity.magnitude > maxInduced)
                {
                    elem.inducedVelocity = elem.inducedVelocity.normalized * maxInduced;
                }
            }
        }
    }
}
