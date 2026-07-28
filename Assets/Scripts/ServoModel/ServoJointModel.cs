using UnityEngine;

/// <summary>
/// Physical servo actuator model for an ArticulationBody joint.
///
/// Simulates a virtual motor rotor (gearbox output side) inside the script:
///   Jm * dwm/dt = tau_servo(PD) - tau_friction(Stribeck, Karnopp stick) - tau_transmission
/// The rotor is coupled to the physical joint through a continuous dead-zone
/// spring-damper (backlash, Modelica ElastoBacklash style):
///   tau_transmission = K * d(delta) + D * s(delta) * (wm - wl),  delta = thetaM - thetaL
///   d(delta) = delta - b * tanh(delta / b)   (smooth dead zone, half width b)
///   s(delta) = tanh^2(delta / b)             (smooth engagement indicator)
/// The spring torque is realized through the PhysX implicit xDrive
/// (stiffness = K * s, target = thetaL + d/s), which stays stable at 50 Hz
/// even for stiff transmissions. The motor rotor is integrated with internal
/// substeps, so stiction and the Stribeck peak are handled exactly.
///
/// All public parameters are SI (rad, rad/s, N*m for revolute; m, m/s, N for
/// prismatic). Unit conversion to the degree-based xDrive happens internally.
/// </summary>
public class ServoJointModel : MonoBehaviour
{
    [Header("Servo internal controller (motor side)")]
    public float servoStiffness = 20f;        // N*m/rad
    public float servoDamping = 0.5f;         // N*m/(rad/s)
    public float motorTorqueLimit = float.MaxValue; // N*m
    public float motorInertia = 2e-3f;        // kg*m^2 (rotor inertia reflected through gearbox)

    [Header("Stribeck friction (motor/gearbox side)")]
    public float staticFriction = 0f;         // N*m, breakaway torque tau_s
    public float dynamicFriction = 0f;        // N*m, Coulomb torque tau_c
    public float stribeckVelocity = 0.1f;     // rad/s, decay velocity of the Stribeck peak
    public float viscousFriction = 0f;        // N*m/(rad/s)

    [Header("Backlash + transmission")]
    public float backlashWidth = 0f;          // rad, TOTAL dead band (gap = backlashWidth, half width b = width/2)
    public float transmissionStiffness = 400f; // N*m/rad
    public float transmissionDamping = 0.5f;  // N*m/(rad/s), only acts while engaged

    [Header("Integration")]
    [Tooltip("0 = choose automatically from stiffness/inertia")]
    public int substeps = 0;

    // Command (SI units, joint space)
    public float targetPosition;
    public float targetVelocity;

    // Read-only state for logging/tests
    public float MotorPosition => m_ThetaM;
    public float MotorVelocity => m_OmegaM;
    public float TransmissionDeflection => m_LastDelta;
    public float TransmissionTorque => m_LastTransTorque;

    ArticulationBody m_Body;
    bool m_Initialized;
    float m_ThetaM;
    float m_OmegaM;
    float m_LastDelta;
    float m_LastTransTorque;
    float m_UnitScale; // SI -> drive units (Rad2Deg for revolute, 1 for prismatic)

    void Awake()
    {
        m_Body = GetComponent<ArticulationBody>();
    }

    void Initialize()
    {
        if (m_Body == null || m_Body.dofCount < 1)
            return;
        m_UnitScale = m_Body.jointType == ArticulationJointType.PrismaticJoint ? 1f : Mathf.Rad2Deg;
        // The articulation must never sleep: a stuck motor keeps the drive
        // target constant, so a slept articulation would stay frozen even
        // with a wound-up transmission spring. Sleeping is decided for the
        // articulation tree, so clear the threshold up to the root.
        foreach (ArticulationBody ab in GetComponentsInParent<ArticulationBody>())
            ab.sleepThreshold = 0f;
        m_ThetaM = m_Body.jointPosition[0];
        m_OmegaM = m_Body.jointVelocity[0];
        targetPosition = m_ThetaM;
        targetVelocity = 0f;
        m_Initialized = true;
    }

    public void SetCommand(float position, float velocity)
    {
        targetPosition = position;
        targetVelocity = velocity;
    }

    int AutoSubsteps(float dt)
    {
        // Semi-implicit Euler needs w0 * h < ~1 for accuracy; w0 is the highest
        // eigenfrequency seen by the rotor (servo PD + transmission spring).
        float k = servoStiffness + transmissionStiffness;
        float w0 = Mathf.Sqrt(k / Mathf.Max(motorInertia, 1e-9f));
        int n = Mathf.CeilToInt(2f * w0 * dt);
        // The Karnopp stick band is tau_s * h / Jm (the velocity friction can
        // cancel in one substep). Keep it below ~ws/4 so slow sliding near the
        // Stribeck knee is resolved instead of forced into stick-slip.
        if (staticFriction > 0f)
        {
            float band = 0.25f * Mathf.Max(stribeckVelocity, 1e-3f);
            int n2 = Mathf.CeilToInt(dt * staticFriction / (Mathf.Max(motorInertia, 1e-9f) * band));
            n = Mathf.Max(n, n2);
        }
        return Mathf.Clamp(n, 4, 400);
    }

    static float Tanh(float x)
    {
        return (float)System.Math.Tanh(x);
    }

    float StribeckTorque(float w)
    {
        float x = stribeckVelocity > 1e-9f ? w / stribeckVelocity : 0f;
        float mag = dynamicFriction + (staticFriction - dynamicFriction) * Mathf.Exp(-x * x);
        return Mathf.Sign(w) * mag + viscousFriction * w;
    }

    void ComputeDeadZone(float delta, out float dz, out float engage)
    {
        float b = 0.5f * backlashWidth;
        if (b > 1e-9f)
        {
            float t = Tanh(delta / b);
            dz = delta - b * t;
            engage = t * t;
        }
        else
        {
            dz = delta;
            engage = 1f;
        }
    }

    void FixedUpdate()
    {
        if (!m_Initialized)
        {
            Initialize();
            if (!m_Initialized)
                return;
        }

        float dt = Time.fixedDeltaTime;
        float thetaL = m_Body.jointPosition[0];
        float omegaL = m_Body.jointVelocity[0];

        int n = substeps > 0 ? substeps : AutoSubsteps(dt);
        float h = dt / n;

        for (int i = 0; i < n; i++)
        {
            // Load state extrapolated linearly across the physics step.
            float thetaLest = thetaL + omegaL * h * i;
            // Real servos interpolate the 50 Hz command internally; without
            // this the zero-order-hold ripple excites the rotor at high speed.
            // The ramp reaches targetPosition at the end of this physics step.
            float cmdPos = targetPosition - targetVelocity * (dt - h * i);

            float tauServo = servoStiffness * (cmdPos - m_ThetaM)
                           + servoDamping * (targetVelocity - m_OmegaM);
            tauServo = Mathf.Clamp(tauServo, -motorTorqueLimit, motorTorqueLimit);

            float delta = m_ThetaM - thetaLest;
            ComputeDeadZone(delta, out float dz, out float engage);
            float tauTrans = transmissionStiffness * dz
                           + transmissionDamping * Mathf.Max(engage, 0.1f) * (m_OmegaM - omegaL);

            float tauNet = tauServo - tauTrans;

            // Karnopp stick-slip: inside the stick band the rotor holds still
            // unless the net torque exceeds the breakaway torque.
            float stickBand = Mathf.Max(1e-4f, staticFriction / Mathf.Max(motorInertia, 1e-9f) * h);
            if (Mathf.Abs(m_OmegaM) < stickBand && Mathf.Abs(tauNet) <= staticFriction)
            {
                m_OmegaM = 0f;
                continue;
            }

            float tauF = Mathf.Abs(m_OmegaM) < stickBand
                ? Mathf.Sign(tauNet) * staticFriction   // breakaway
                : StribeckTorque(m_OmegaM);

            m_OmegaM += h * (tauNet - tauF) / Mathf.Max(motorInertia, 1e-9f);
            m_ThetaM += h * m_OmegaM;
        }

        // Realize the transmission through the implicit xDrive so that stiff
        // springs stay stable at coarse fixed timesteps. The stiffness is kept
        // constant at K and the backlash enters through the anchor position:
        //   target = thetaM - b*tanh(delta/b)
        // so the start-of-step torque is K*(target - thetaL) = K*d(delta).
        // Keeping K always on is essential: fading stiffness to zero inside the
        // gap lets the load penetrate a flank within one step and the re-wound
        // spring then injects energy out of nowhere (numeric restitution > 1).
        // The cost is that free in-gap motion is only resolved down to the
        // physics step duration, which is the right trade-off at 50 Hz.
        // Evaluate the spring against the PREDICTED end-of-step load position.
        // The drive target is held (ZOH) while the load moves omegaL*dt during
        // the step; anchoring at the start-of-step position would bias the
        // deflection by omegaL*dt, which exceeds the static deflection tau/K
        // for stiff transmissions and flips the perceived contact flank.
        float thetaLend = thetaL + omegaL * dt;
        float deltaEnd = m_ThetaM - thetaLend;
        ComputeDeadZone(deltaEnd, out float dzEnd, out float engageEnd);
        float dampScale = Mathf.Max(engageEnd, 0.1f);
        m_LastDelta = deltaEnd;
        m_LastTransTorque = transmissionStiffness * dzEnd
                          + transmissionDamping * dampScale * (m_OmegaM - omegaL);

        // Unit note (verified empirically): xDrive target/targetVelocity are in
        // degrees for rotational drives, but stiffness/damping act on the error
        // converted to radians, i.e. their units are N*m/rad — pass SI directly.
        var drive = m_Body.xDrive;
        drive.stiffness = transmissionStiffness;
        drive.target = (thetaLend + dzEnd) * m_UnitScale; // == thetaM - b*tanh(delta/b)
        drive.damping = transmissionDamping * dampScale;
        drive.targetVelocity = m_OmegaM * m_UnitScale;
        drive.forceLimit = float.MaxValue;
        m_Body.xDrive = drive;
        if (m_Body.IsSleeping())
            m_Body.WakeUp();
    }
}
