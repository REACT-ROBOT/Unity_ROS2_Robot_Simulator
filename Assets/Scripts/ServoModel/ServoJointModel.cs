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
    float m_PrevThetaL;
    bool m_HasPrevThetaL;
    float m_LimLower0;
    float m_LimUpper0;

#if UNITY_EDITOR
    const float DriveUnitCompensation = 1f;
#else
    static readonly float DriveUnitCompensation = Mathf.Rad2Deg;
#endif

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
        // Batch/headless runs can report non-finite joint state on the very
        // first frame; a NaN here would poison the drive forever ("The
        // supplied joint drive has non-finite values").
        if (!float.IsFinite(m_ThetaM))
            return; // retry next FixedUpdate
        // jointVelocity is not sampled here: see the note in FixedUpdate.
        m_OmegaM = 0f;
        targetPosition = m_ThetaM;
        targetVelocity = 0f;
        // Configure the constant drive fields once.
        // Unit quirk: in STANDALONE PLAYER builds (measured on the Linux
        // player, Unity 6000.2) rotational xDrive stiffness/damping/
        // forceLimit are effectively scaled by pi/180 relative to their SI
        // meaning (a drive with stiffness S holds with S*pi/180 N*m/rad;
        // verified from the static sag of a plain position drive under a
        // known gravity torque). In the EDITOR the same fields act as plain
        // N*m/rad (verified by the ServoPendulumTests friction-curve fit,
        // accurate to <1%). Compensate only in player builds so the public
        // fields keep their SI units in both environments.
        var drive0 = m_Body.xDrive;
        drive0.stiffness = transmissionStiffness * DriveUnitCompensation;
        drive0.damping = transmissionDamping * DriveUnitCompensation;
        drive0.forceLimit = 1e3f * DriveUnitCompensation;
        m_Body.xDrive = drive0;
        m_LimLower0 = drive0.lowerLimit;
        m_LimUpper0 = drive0.upperLimit;
        m_Initialized = true;
    }

    public void SetCommand(float position, float velocity)
    {
        targetPosition = position;
        targetVelocity = velocity;
    }

    /// <summary>
    /// モデル内部の状態をスポーン直後と同じ状態へ戻す。
    /// </summary>
    /// <remarks>
    /// reset_simulation で関節の位置と速度を戻すだけでは足りない。ロータ角
    /// (m_ThetaM)、伝達ばねのたわみ、指令値といったモデル側の状態が残っていると、
    /// 巻き上がったトルクがリセット直後に解放されて関節が跳ねる。ここは関節を
    /// ゼロへ戻した「後」に呼ぶこと (ロータ角を関節の現在値に合わせるため)。
    /// </remarks>
    public void ResetState()
    {
        if (m_Body == null)
            m_Body = GetComponent<ArticulationBody>();

        float theta = 0f;
        if (m_Body != null && m_Body.dofCount > 0)
        {
            float current = m_Body.jointPosition[0];
            if (float.IsFinite(current))
                theta = current;
        }

        m_ThetaM = theta;
        m_OmegaM = 0f;
        targetPosition = theta;
        targetVelocity = 0f;
        m_LastDelta = 0f;
        m_LastTransTorque = 0f;
        // 有限差分による負荷速度の履歴。残すとリセット直後の 1 ステップで
        // 巨大な速度差として効いてしまう。
        m_PrevThetaL = theta;
        m_HasPrevThetaL = false;
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

        // NOTE on joint limits (this engine build): a joint that comes to
        // rest exactly ON its limit can stop responding to drive torques,
        // and rewriting the drive limits at runtime to release it makes the
        // joint position SNAP (hundreds of rad/s), so no un-jam is attempted
        // here. Instead the robot should be designed so that the rest pose
        // (gravity equilibrium) sits well inside the joint range and the
        // limits stay out of normal reach; the rotor clamp below keeps the
        // transmission from pressing the joint against a limit.

        // Load velocity source differs per environment:
        // - EDITOR: jointVelocity is clean and current -> use it directly
        //   (a finite difference would add one step of delay, which
        //   destabilizes stiff transmissions; verified: the pendulum tests
        //   explode with FD at K = 400 N*m/rad).
        // - PLAYER: jointVelocity of a runtime-imported joint held static
        //   under gravity reports the pre-solve gravity step
        //   (~ tau_g / J * dt) instead of zero. That bias feeds the load
        //   extrapolation below and turns the transmission into positive
        //   feedback (the joint runs away and pins at a limit). Use a
        //   bias-free position finite difference instead and keep the
        //   transmission stiffness low enough for the added delay (see the
        //   stability note in docs/Servo-Model-Guide.md).
#if UNITY_EDITOR
        float omegaL = m_Body.jointVelocity[0];
        m_PrevThetaL = thetaL;
        m_HasPrevThetaL = true;
#else
        float omegaL = m_HasPrevThetaL ? (thetaL - m_PrevThetaL) / dt : 0f;
        m_PrevThetaL = thetaL;
        m_HasPrevThetaL = true;
#endif

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

        // The virtual rotor cannot pass the joint end stops either (a real
        // horn is stopped by the case). Without this clamp a violent swing
        // drags the rotor beyond the limit and the transmission then pushes
        // the joint outward against its limit forever.
        if (m_Body.jointType != ArticulationJointType.PrismaticJoint
            && m_Body.twistLock == ArticulationDofLock.LimitedMotion)
        {
            float rotLo = (m_LimLower0 + 1f) * Mathf.Deg2Rad;
            float rotHi = (m_LimUpper0 - 1f) * Mathf.Deg2Rad;
            if (m_ThetaM < rotLo)
            {
                m_ThetaM = rotLo;
                if (m_OmegaM < 0f) m_OmegaM = 0f;
            }
            else if (m_ThetaM > rotHi)
            {
                m_ThetaM = rotHi;
                if (m_OmegaM > 0f) m_OmegaM = 0f;
            }
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

        // Unit note (verified empirically on the Linux player, Unity 6):
        // xDrive target is in degrees for rotational drives, but stiffness and
        // damping act on the error converted to radians (units N*m/rad), and
        // targetVelocity is consumed in rad/s by the damping term. Passing
        // deg/s here multiplies the relative-damping force by ~57, which turns
        // the transmission damping into a violent energy pump (the joint
        // oscillates full-range; large damping values diverge to infinity).
        // Only target/targetVelocity change per frame; stiffness, damping and
        // forceLimit were configured once in Initialize (see note there).
        // target/targetVelocity use degree units, which the engine converts
        // correctly (verified: a plain drive tracks its target 1:1).
        var drive = m_Body.xDrive;
        drive.target = (thetaLend + dzEnd) * m_UnitScale; // == thetaM - b*tanh(delta/b)
        drive.targetVelocity = m_OmegaM * m_UnitScale;
        m_Body.xDrive = drive;
        if (m_Body.IsSleeping())
            m_Body.WakeUp();
    }
}
