// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// Unsteady aerodynamic effects for blade element theory
// Implements added mass and Wagner function circulation lag

using UnityEngine;

namespace Aerodynamics.BladeElement
{
    /// <summary>
    /// Computes unsteady aerodynamic effects for blade elements.
    /// Includes added mass (virtual mass) and Wagner function circulation lag.
    /// </summary>
    public class UnsteadyEffects
    {
        private AeroSurfaceParameters parameters;

        /// <summary>
        /// Initialize with surface parameters
        /// </summary>
        public void Initialize(AeroSurfaceParameters params_)
        {
            this.parameters = params_;
        }

        /// <summary>
        /// Compute all unsteady effects for a blade element
        /// </summary>
        /// <param name="elem">Blade element state</param>
        /// <param name="density">Fluid density in kg/m³</param>
        /// <param name="dt">Time step in seconds</param>
        /// <returns>Additional force and torque from unsteady effects</returns>
        public BiVector3 ComputeUnsteadyForces(BladeElementState elem, float density, float dt)
        {
            BiVector3 result = BiVector3.zero;

            if (parameters == null || !parameters.enableUnsteadyEffects)
                return result;

            // Added mass force
            if (parameters.enableAddedMass)
            {
                Vector3 addedMassForce = ComputeAddedMassForce(elem, density, dt);
                result.force += addedMassForce;
                elem.lastAddedMassForce = addedMassForce;
            }

            return result;
        }

        /// <summary>
        /// Compute Wagner factor for circulation lag
        /// Returns a factor (0-1) that reduces lift during transients
        /// </summary>
        /// <param name="elem">Blade element state</param>
        /// <param name="dt">Time step in seconds</param>
        /// <returns>Wagner factor (1 = steady state, &lt;1 = transient)</returns>
        public float ComputeWagnerFactor(BladeElementState elem, float dt)
        {
            if (parameters == null || !parameters.enableUnsteadyEffects || !parameters.enableWagnerLag)
                return 1f;

            float V = elem.currentVelocity.magnitude;
            float c = elem.localChord;

            if (V < 0.1f || c < 0.001f)
                return 1f;

            // Wagner function approximation (Jones 1938):
            // Φ(s) = 1 - 0.165*exp(-0.0455*s) - 0.335*exp(-0.3*s)
            // where s = 2*V*t/c (semi-chords traveled)

            // For real-time simulation, use first-order lag approximation:
            // dΓ/dt = (Γ_qs - Γ) / τ
            // where τ = wagnerTimeConstant * c / V

            float tau = parameters.wagnerTimeConstant * c / V;
            tau = Mathf.Max(tau, dt);  // Prevent division issues

            // Update semi-chords traveled
            float ds = 2f * V * dt / c;
            elem.semiChordsTraveled += ds;

            // Quasi-steady circulation (proportional to Cl * V * c)
            float Gamma_qs = 0.5f * parameters.liftSlope * elem.currentAoA * V * c;
            elem.circulationQuasiSteady = Gamma_qs;

            // First-order lag for circulation
            float alpha = dt / (tau + dt);
            alpha = Mathf.Clamp01(alpha);

            elem.circulationLag = Mathf.Lerp(elem.circulationLag, Gamma_qs, alpha);

            // Wagner factor = lagged / quasi-steady
            if (Mathf.Abs(Gamma_qs) > 0.001f)
            {
                float wagnerFactor = elem.circulationLag / Gamma_qs;
                return Mathf.Clamp(wagnerFactor, 0.1f, 1.5f);  // Clamp for stability
            }

            return 1f;
        }

        /// <summary>
        /// Compute full Wagner function value (for analysis/comparison)
        /// </summary>
        /// <param name="semiChords">Number of semi-chords traveled</param>
        /// <returns>Wagner function value Φ(s)</returns>
        public static float WagnerFunction(float semiChords)
        {
            // Jones (1938) approximation:
            // Φ(s) = 1 - 0.165*exp(-0.0455*s) - 0.335*exp(-0.3*s)
            float s = Mathf.Max(semiChords, 0f);
            return 1f - 0.165f * Mathf.Exp(-0.0455f * s) - 0.335f * Mathf.Exp(-0.3f * s);
        }

        /// <summary>
        /// Compute added mass (virtual mass) force
        /// </summary>
        /// <param name="elem">Blade element state</param>
        /// <param name="density">Fluid density in kg/m³</param>
        /// <param name="dt">Time step in seconds</param>
        /// <returns>Added mass force vector</returns>
        private Vector3 ComputeAddedMassForce(BladeElementState elem, float density, float dt)
        {
            if (dt <= 0f)
                return Vector3.zero;

            float c = elem.localChord;
            float b = elem.elementSpan;
            float k = parameters.addedMassCoefficient;

            // Added mass per unit span for thin airfoil:
            // m_a = π * ρ * c² / 4
            // For 3D element: M_a = m_a * b * k
            float m_added = density * Mathf.PI * c * c / 4f * b * k;

            // Acceleration (perpendicular to chord is most significant)
            Vector3 acceleration = elem.GetAcceleration(dt);

            // Added mass force opposes acceleration
            Vector3 F_am = -m_added * acceleration;

            // Validate and clamp
            if (float.IsNaN(F_am.x) || float.IsInfinity(F_am.x))
                return Vector3.zero;

            // Limit to reasonable values (prevent numerical explosion)
            float maxForce = density * elem.currentVelocity.sqrMagnitude * elem.elementArea * 10f;
            if (F_am.magnitude > maxForce)
            {
                F_am = F_am.normalized * maxForce;
            }

            return F_am;
        }

        /// <summary>
        /// Compute added mass moment (for pitching)
        /// </summary>
        /// <param name="elem">Blade element state</param>
        /// <param name="density">Fluid density in kg/m³</param>
        /// <param name="pitchRate">Pitch rate in rad/s</param>
        /// <param name="dt">Time step in seconds</param>
        /// <returns>Added mass moment (about span axis)</returns>
        public float ComputeAddedMassMoment(BladeElementState elem, float density, float pitchRate, float dt)
        {
            if (parameters == null || !parameters.enableUnsteadyEffects || !parameters.enableAddedMass)
                return 0f;

            float c = elem.localChord;
            float b = elem.elementSpan;
            float k = parameters.addedMassCoefficient;

            // Added moment of inertia about quarter-chord:
            // I_a = π * ρ * c⁴ / 128
            // For 3D element: I_a * b * k
            float I_added = density * Mathf.PI * c * c * c * c / 128f * b * k;

            // Pitch acceleration (approximate from rate change)
            float pitchAccel = elem.GetAoARate(dt);

            // Added mass moment opposes angular acceleration
            float M_am = -I_added * pitchAccel;

            // Clamp for stability
            float maxMoment = density * elem.currentVelocity.sqrMagnitude * elem.elementArea * c;
            M_am = Mathf.Clamp(M_am, -maxMoment, maxMoment);

            return M_am;
        }

        /// <summary>
        /// Compute Theodorsen-like deficiency function (simplified)
        /// Returns a complex number as (real, imaginary) for phase lag
        /// </summary>
        /// <param name="reducedFrequency">k = ω*c / (2*V)</param>
        /// <returns>Magnitude of Theodorsen function C(k)</returns>
        public static float TheodorsenMagnitude(float reducedFrequency)
        {
            // Approximation from Jones (1938):
            // |C(k)| ≈ 1 / sqrt(1 + (π*k)²)
            // Valid for k < 1 (typical for most applications)
            float k = Mathf.Abs(reducedFrequency);
            return 1f / Mathf.Sqrt(1f + Mathf.PI * Mathf.PI * k * k);
        }

        /// <summary>
        /// Compute phase lag from Theodorsen function
        /// </summary>
        /// <param name="reducedFrequency">k = ω*c / (2*V)</param>
        /// <returns>Phase lag in radians</returns>
        public static float TheodorsenPhase(float reducedFrequency)
        {
            // Approximation: φ ≈ -atan(π*k)
            float k = Mathf.Abs(reducedFrequency);
            return -Mathf.Atan(Mathf.PI * k);
        }
    }
}
