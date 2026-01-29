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
    /// Prandtl lifting-line theory for finite wings
    /// Best for fixed-wing aircraft
    /// Uses proper induced velocity formulation with Oswald efficiency
    /// </summary>
    public class PrandtlInducedVelocity : IInducedVelocityModel
    {
        private float aspectRatio;
        private float oswaldEfficiency;

        public void Initialize(AeroSurfaceParameters parameters)
        {
            aspectRatio = parameters.EffectiveAspectRatio;
            // Estimate Oswald efficiency based on taper ratio
            // e ≈ 1.78 * (1 - 0.045 * AR^0.68) - 0.64 for swept wings
            // Simplified: e ≈ 0.85-0.95 for well-designed wings
            float taper = parameters.taperRatio;
            // Empirical formula: e = 1 / (1 + 0.03 + 0.005*AR + delta_e(taper))
            // For taper: optimal taper ~0.45, efficiency drops for rectangular or highly tapered
            float taperPenalty = 0.1f * Mathf.Abs(taper - 0.45f);
            oswaldEfficiency = Mathf.Clamp(0.95f - 0.02f * aspectRatio - taperPenalty, 0.7f, 0.98f);
        }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            if (elements == null || elements.Length == 0) return;

            float ar = parameters.EffectiveAspectRatio;
            float span = parameters.span;

            // First pass: compute total wing lift coefficient
            float totalCl = 0f;
            float totalArea = 0f;
            foreach (var elem in elements)
            {
                totalCl += elem.currentCl * elem.elementArea;
                totalArea += elem.elementArea;
            }
            float avgCl = totalArea > 0.001f ? totalCl / totalArea : 0f;

            for (int i = 0; i < elements.Length; i++)
            {
                BladeElementState elem = elements[i];

                // Spanwise position: 0 at root, 1 at tip (or 0 at center, 1 at tips for symmetric)
                // Convert spanwiseFraction to normalized distance from centerline
                // spanwiseFraction: 0 = root, 1 = tip
                float eta = elem.spanwiseFraction;  // 0 to 1 from root to tip

                // For elliptic loading, circulation distribution: Γ(y) = Γ_0 * sqrt(1 - (2y/b)^2)
                // where y is distance from centerline, b is full span
                // For a half-wing with eta = 0 at root, 1 at tip:
                // normalized position from centerline = eta
                float y_norm = eta;  // 0 at root (centerline), 1 at tip

                // Elliptic loading factor (1 at root, 0 at tip)
                float ellipticFactor = Mathf.Sqrt(Mathf.Max(1f - y_norm * y_norm, 0.01f));

                // Tip-loss factor for finite wing
                // Based on spanwise lift distribution, not propeller tip-loss
                // F = ellipticFactor for ideal elliptic wing
                // Add correction for non-elliptic loading due to taper
                float taper = parameters.taperRatio;
                float loadingCorrection = 1f - 0.2f * Mathf.Abs(taper - 0.45f) * (1f - y_norm);
                float F = ellipticFactor * loadingCorrection;
                F = Mathf.Clamp(F, 0.05f, 1f);
                elem.tipLossFactor = F;

                // Induced velocity from lifting-line theory:
                // For elliptic loading: α_i = C_L / (π * AR)  (induced angle of attack)
                // Therefore: w_i = V * α_i = V * C_L / (π * AR)
                //
                // For non-elliptic loading with Oswald efficiency:
                // α_i = C_L / (π * AR * e)
                // w_i = V * C_L / (π * AR * e)
                //
                // Local variation: w_i(y) = w_i_avg / ellipticFactor for uniform downwash
                // Or use actual local Cl for more accurate distribution

                float V = elem.currentVelocity.magnitude;
                float Cl = elem.currentCl;

                if (V > 0.1f && ar > 0.1f)
                {
                    // Use local Cl for induced velocity at this element
                    // w_i = V * Cl / (π * AR * e)
                    float w_i_local = V * Cl / (Mathf.PI * ar * oswaldEfficiency);

                    // Apply spanwise distribution correction
                    // Near tips, induced velocity is higher due to tip vortex
                    // w_i_tip_corrected = w_i * (1 + k * (1 - F))
                    // where k accounts for tip vortex concentration
                    float tipVortexFactor = 1f + 0.3f * (1f - F);
                    w_i_local *= tipVortexFactor;

                    // Clamp to reasonable values (induced velocity shouldn't exceed ~30% of freestream)
                    w_i_local = Mathf.Clamp(w_i_local, 0f, V * 0.3f);

                    // Induced velocity is downward (negative Y in typical orientation)
                    // Sign follows convention: positive Cl produces negative w_i (downwash)
                    float sign = Cl >= 0 ? -1f : 1f;
                    elem.inducedVelocity = new Vector3(0f, sign * Mathf.Abs(w_i_local), 0f);
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
    /// Supports both hover and forward flight conditions
    /// </summary>
    public class MomentumInducedVelocity : IInducedVelocityModel
    {
        private const int MAX_ITERATIONS = 10;
        private const float CONVERGENCE_TOLERANCE = 0.001f;
        private const int NUM_BLADES = 2;  // Default blade count for tip-loss

        public void Initialize(AeroSurfaceParameters parameters) { }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            if (elements == null || elements.Length == 0) return;

            float span = parameters.span;
            float density = parameters.airDensity;  // TODO: support water

            // Sum thrust from all elements to get total
            float totalThrust = 0f;
            float avgAxialVelocity = 0f;
            int validElements = 0;

            foreach (var elem in elements)
            {
                // Approximate thrust from lift coefficient
                float q = elem.GetDynamicPressure(density);
                float thrust = elem.currentCl * q * elem.elementArea;
                totalThrust += Mathf.Abs(thrust);

                // Average axial velocity (Y component in local frame)
                avgAxialVelocity += elem.currentVelocity.y;
                validElements++;
            }

            if (validElements > 0)
            {
                avgAxialVelocity /= validElements;
            }

            // Disk area (assuming circular rotor)
            float diskArea = Mathf.PI * span * span / 4f;
            if (diskArea < 0.01f) diskArea = 0.01f;

            // Hover induced velocity (reference)
            float v_i_hover = Mathf.Sqrt(Mathf.Max(totalThrust, 0.001f) / (2f * density * diskArea));

            // Forward flight correction using Glauert's formula
            // For climb/descent: T = 2 * rho * A * v_i * sqrt((V_climb)^2 + (v_i)^2)
            // This requires solving for v_i iteratively or using approximation
            float V_climb = -avgAxialVelocity;  // Positive V_climb = climbing (opposing induced flow)

            float v_i_avg;
            if (Mathf.Abs(V_climb) < 0.1f * v_i_hover)
            {
                // Near hover - use hover value
                v_i_avg = v_i_hover;
            }
            else if (V_climb > 0)
            {
                // Climb - solve T = 2*rho*A*v_i*sqrt(V_c^2 + v_i^2)
                // Let x = v_i/v_i_hover, mu = V_climb/v_i_hover
                // Then: 1 = x * sqrt(mu^2 + x^2)
                // Approximation: v_i = v_i_hover * (sqrt(1 + (V_c/2/v_i_hover)^2) - V_c/2/v_i_hover)
                float mu = V_climb / (2f * v_i_hover);
                v_i_avg = v_i_hover * (Mathf.Sqrt(1f + mu * mu) - mu);
            }
            else
            {
                // Descent - more complex (vortex ring state possible)
                // Use empirical approximation from Leishman
                float V_d = -V_climb;
                float v_ratio = V_d / v_i_hover;

                if (v_ratio < 1.5f)
                {
                    // Vortex ring state - induced velocity increases
                    v_i_avg = v_i_hover * (1f + 0.5f * v_ratio);
                }
                else if (v_ratio < 2.0f)
                {
                    // Turbulent wake state
                    v_i_avg = v_i_hover * (2.5f - 0.5f * v_ratio);
                }
                else
                {
                    // Windmill brake state: v_i = V_d/2 - sqrt((V_d/2)^2 - v_i_hover^2)
                    float temp = (V_d / 2f) * (V_d / 2f) - v_i_hover * v_i_hover;
                    if (temp > 0)
                    {
                        v_i_avg = V_d / 2f - Mathf.Sqrt(temp);
                    }
                    else
                    {
                        v_i_avg = v_i_hover * 0.5f;
                    }
                }
            }

            // Clamp to reasonable values
            v_i_avg = Mathf.Clamp(v_i_avg, 0.01f, v_i_hover * 3f);

            // Apply to each element with radial variation and Prandtl tip-loss
            for (int i = 0; i < elements.Length; i++)
            {
                BladeElementState elem = elements[i];

                // Radial position (0 = root, 1 = tip)
                float r_R = elem.spanwiseFraction;
                r_R = Mathf.Clamp(r_R, 0.01f, 0.99f);

                // Calculate inflow angle at this element
                // phi = atan(V_climb + v_i) / (omega * r) ≈ atan(v_z / v_x)
                float Vz = elem.currentVelocity.z;  // In-plane velocity
                float Vy = elem.currentVelocity.y + v_i_avg;  // Axial + induced
                float phi = Mathf.Atan2(Mathf.Abs(Vy), Mathf.Abs(Vz) + 0.1f);

                // Prandtl tip-loss factor with inflow angle
                // f = (B/2) * (1 - r/R) / (r/R * sin(phi))
                float sinPhi = Mathf.Max(Mathf.Sin(phi), 0.05f);
                float f_tip = (NUM_BLADES / 2f) * (1f - r_R) / (r_R * sinPhi);
                float F = (2f / Mathf.PI) * Mathf.Acos(Mathf.Clamp(Mathf.Exp(-f_tip), -1f, 1f));
                F = Mathf.Clamp(F, 0.05f, 1f);
                elem.tipLossFactor = F;

                // Local induced velocity
                // For uniform loading: v_i_local = v_i_avg / F (increases toward tip)
                // For more realistic distribution: v_i_local = v_i_avg * sqrt(F)
                float v_i_local = v_i_avg / Mathf.Sqrt(F);

                // Induced velocity is perpendicular to disk plane (Unity: -Y for downwash)
                elem.inducedVelocity = new Vector3(0f, -v_i_local, 0f);
            }
        }
    }

    /// <summary>
    /// Simple wake model for flapping wings
    /// Uses quasi-steady approximation with time-lagged wake effects
    /// Based on Theodorsen's unsteady thin airfoil theory simplified for flapping
    /// </summary>
    public class SimpleWakeInducedVelocity : IInducedVelocityModel
    {
        private Vector3[] previousInducedVelocities;
        private float[] wakeStrength;  // Accumulated wake circulation
        private const float WAKE_TIME_CONSTANT = 0.15f;  // Time constant for wake development (seconds)
        private const float WAKE_DECAY_TIME = 0.5f;  // Time for wake to decay to 37% (seconds)

        public void Initialize(AeroSurfaceParameters parameters) { }

        public void ComputeInducedVelocities(BladeElementState[] elements, AeroSurfaceParameters parameters, float dt)
        {
            if (elements == null || elements.Length == 0) return;

            // Initialize arrays if needed
            if (previousInducedVelocities == null || previousInducedVelocities.Length != elements.Length)
            {
                previousInducedVelocities = new Vector3[elements.Length];
                wakeStrength = new float[elements.Length];
            }

            float span = parameters.span;
            float ar = parameters.EffectiveAspectRatio;

            for (int i = 0; i < elements.Length; i++)
            {
                BladeElementState elem = elements[i];

                // Tip loss factor (quadratic distribution for flapping wings)
                float eta = elem.spanwiseFraction;  // 0 at root, 1 at tip
                elem.tipLossFactor = 1f - 0.35f * eta * eta;  // Higher tip loss for flapping

                // Quasi-steady induced velocity using lifting-line approximation
                float V = elem.currentVelocity.magnitude;
                float Cl = elem.currentCl;

                float w_i_qs = 0f;
                if (V > 0.1f && ar > 0.1f)
                {
                    // Similar to Prandtl but with flapping correction
                    // Flapping wings have less efficient span loading due to unsteady effects
                    float flappingEfficiency = 0.75f;  // Reduced efficiency for flapping
                    w_i_qs = V * Cl / (Mathf.PI * ar * flappingEfficiency);
                }

                Vector3 w_i_target = new Vector3(0f, -w_i_qs, 0f);

                // First-order lag model for wake development
                // dv_i/dt = (v_i_qs - v_i) / tau
                // Solution: v_i(t) = v_i_qs + (v_i_0 - v_i_qs) * exp(-t/tau)
                // Discrete: v_i_new = v_i_old + (v_i_qs - v_i_old) * (1 - exp(-dt/tau))
                float developmentFactor = 1f - Mathf.Exp(-dt / WAKE_TIME_CONSTANT);
                Vector3 w_i_developed = previousInducedVelocities[i] + (w_i_target - previousInducedVelocities[i]) * developmentFactor;

                // Wake memory effect: shed vortices continue to affect the wing
                // Model as accumulated wake circulation that decays over time
                // This accounts for returning wake in flapping motion
                float wakeDecayFactor = Mathf.Exp(-dt / WAKE_DECAY_TIME);
                float newWakeContribution = Mathf.Abs(Cl) * V * 0.1f;  // Circulation shed this timestep
                wakeStrength[i] = wakeStrength[i] * wakeDecayFactor + newWakeContribution * dt / WAKE_DECAY_TIME;

                // Wake-induced velocity adds to the quasi-steady value
                // Direction is same as quasi-steady (downwash for positive lift)
                float wakeInducedMagnitude = wakeStrength[i] * 0.2f;  // Scaling factor
                Vector3 wakeInduced = new Vector3(0f, -Mathf.Sign(Cl) * wakeInducedMagnitude, 0f);

                elem.inducedVelocity = w_i_developed + wakeInduced;

                // Store for next frame
                previousInducedVelocities[i] = w_i_developed;

                // Clamp to reasonable values
                // For flapping, allow higher induced velocities due to vortex interactions
                float maxInduced = Mathf.Max(V * 0.6f, 1f);
                if (elem.inducedVelocity.magnitude > maxInduced)
                {
                    elem.inducedVelocity = elem.inducedVelocity.normalized * maxInduced;
                }
            }
        }
    }
}
