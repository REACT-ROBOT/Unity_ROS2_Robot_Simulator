using System.Collections;
using System.Globalization;
using System.IO;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Single-pendulum validation of ServoJointModel.
/// Each test writes a CSV to &lt;project&gt;/TestOutput/ for offline analysis.
///
/// Rig: immovable base + one revolute arm (twist about X), point-mass-like
/// pendulum (m = 0.2 kg, L = 0.2 m, max gravity torque ~0.39 N*m).
/// </summary>
public class ServoPendulumTests
{
    const float Mass = 0.2f;
    const float Length = 0.2f;
    const float Kp = 20f;       // servo internal P gain [N*m/rad]
    const float Kd = 0.5f;      // servo internal D gain [N*m/(rad/s)]
    const float TauMax = 2f;    // motor torque limit [N*m]
    const float Jm = 2e-3f;     // reflected rotor inertia [kg*m^2]
    const float TauS = 0.15f;   // breakaway torque [N*m]
    const float TauC = 0.08f;   // Coulomb torque [N*m]
    const float Ws = 0.1f;      // Stribeck velocity [rad/s]
    const float Sigma = 0.005f; // viscous [N*m/(rad/s)]
    const float Gap = 2f * 0.0087f; // total backlash 1 deg
    // Transmission stiffness for the backlash tests. Two competing bounds fix
    // this at 50 Hz. The gap only dominates the deflection above K = tau/b, so
    // the backlash tests need a stiff transmission (K > 25 for the gravity
    // crossing, K > 6 for the hysteresis loop). Against that, the dead-zone
    // argument is one step of load travel stale, which reaches the joint as
    // K*omega*dt, so the model stays quantitative only while that is small
    // against the torque under test (K << 73 and K << 33 respectively).
    // K = 50 sits inside both windows; the old K = 400 was outside them and
    // outside the stability limit as well.
    const float Ktrans = 50f;   // transmission stiffness [N*m/rad]
    // The friction sweep runs up to 2 rad/s -- 13x faster than any other test
    // -- so its K*omega*dt bound is 13x tighter and no single value serves
    // both. That test already zeroes the backlash to isolate the Stribeck
    // curve; it decouples the transmission for the same reason. Measured
    // recovery error across the whole sweep: 44 % at K = 2 with the
    // transmission term missing from the balance, 7.7 % once the balance is
    // complete, 0.8 % at K = 0.2.
    const float KtransFriction = 0.2f; // N*m/rad, used by FrictionCurve only
    const float Dtrans = 0.5f;  // transmission damping [N*m/(rad/s)]

    GameObject m_Root;
    ArticulationBody m_Arm;
    ServoJointModel m_Servo;
    Vector3 m_SavedGravity;
    float m_SavedTimeScale;
    float m_SavedMaxDelta;

    [SetUp]
    public void SetUp()
    {
        m_SavedGravity = Physics.gravity;
        m_SavedTimeScale = Time.timeScale;
        m_SavedMaxDelta = Time.maximumDeltaTime;
        Time.fixedDeltaTime = 0.02f;
        Time.timeScale = 10f;
        Time.maximumDeltaTime = 1f;
    }

    [TearDown]
    public void TearDown()
    {
        if (m_Root != null)
            Object.Destroy(m_Root);
        Physics.gravity = m_SavedGravity;
        Time.timeScale = m_SavedTimeScale;
        Time.maximumDeltaTime = m_SavedMaxDelta;
    }

    void CreatePendulum(float jointFriction = 0f)
    {
        m_Root = new GameObject("servo_test_base");
        var baseBody = m_Root.AddComponent<ArticulationBody>();
        baseBody.immovable = true;

        var arm = new GameObject("servo_test_arm");
        arm.transform.SetParent(m_Root.transform, false);
        m_Arm = arm.AddComponent<ArticulationBody>();
        m_Arm.jointType = ArticulationJointType.RevoluteJoint;
        m_Arm.twistLock = ArticulationDofLock.FreeMotion;
        m_Arm.anchorPosition = Vector3.zero;
        m_Arm.anchorRotation = Quaternion.identity;
        m_Arm.mass = Mass;
        m_Arm.automaticCenterOfMass = false;
        m_Arm.centerOfMass = new Vector3(0f, -Length, 0f);
        m_Arm.automaticInertiaTensor = false;
        m_Arm.inertiaTensor = new Vector3(2e-3f, 2e-3f, 2e-3f);
        m_Arm.inertiaTensorRotation = Quaternion.identity;
        m_Arm.jointFriction = jointFriction;
        m_Arm.linearDamping = 0f;
        m_Arm.angularDamping = 0f;
        m_Arm.useGravity = true;

        m_Servo = arm.AddComponent<ServoJointModel>();
        m_Servo.servoStiffness = Kp;
        m_Servo.servoDamping = Kd;
        m_Servo.motorTorqueLimit = TauMax;
        m_Servo.motorInertia = Jm;
        m_Servo.staticFriction = TauS;
        m_Servo.dynamicFriction = TauC;
        m_Servo.stribeckVelocity = Ws;
        m_Servo.viscousFriction = Sigma;
        m_Servo.backlashWidth = Gap;
        m_Servo.transmissionStiffness = Ktrans;
        m_Servo.transmissionDamping = Dtrans;
    }

    static string OutputDir()
    {
        string dir = Path.Combine(Application.dataPath, "..", "TestOutput");
        Directory.CreateDirectory(dir);
        return dir;
    }

    static void AppendRow(StringBuilder sb, params float[] vals)
    {
        for (int i = 0; i < vals.Length; i++)
        {
            if (i > 0) sb.Append(',');
            sb.Append(vals[i].ToString("G9", CultureInfo.InvariantCulture));
        }
        sb.Append('\n');
    }

    void Log(StringBuilder sb, float t, float cmd)
    {
        AppendRow(sb, t, cmd, m_Arm.jointPosition[0], m_Arm.jointVelocity[0],
            m_Servo.MotorPosition, m_Servo.MotorVelocity,
            m_Servo.TransmissionDeflection, m_Servo.TransmissionTorque,
            m_Arm.driveForce.dofCount > 0 ? m_Arm.driveForce[0] : float.NaN,
            m_Arm.xDrive.target);
    }

    const string Header = "t,cmd,thetaL,omegaL,thetaM,omegaM,delta,tauTrans,driveForce,driveTarget\n";

    /// <summary>
    /// Slow triangle wave without gravity. Load resistance comes from
    /// ArticulationBody.jointFriction. The cmd-vs-thetaL plot must show a
    /// hysteresis loop of roughly the backlash width.
    /// </summary>
    [UnityTest]
    public IEnumerator Hysteresis_TriangleWave_ShowsBacklashLoop()
    {
        Physics.gravity = Vector3.zero;
        CreatePendulum(jointFriction: 0.05f);
        var sb = new StringBuilder(Header);

        const float amp = 0.3f;
        const float period = 16f;
        const float settle = 2f;
        int settleSteps = Mathf.RoundToInt(settle / 0.02f);
        int steps = Mathf.RoundToInt(2f * period / 0.02f);

        for (int i = 0; i < settleSteps; i++)
        {
            m_Servo.SetCommand(0f, 0f);
            yield return new WaitForFixedUpdate();
        }

        float risingCross = float.NaN, fallingCross = float.NaN;
        float prevCmd = 0f;
        for (int i = 0; i < steps; i++)
        {
            float t = i * 0.02f;
            // triangle: 0 -> amp -> -amp -> 0 over one period
            float phase = t / period - Mathf.Floor(t / period);
            float cmd = phase < 0.25f ? 4f * amp * phase
                      : phase < 0.75f ? amp - 4f * amp * (phase - 0.25f)
                      : -amp + 4f * amp * (phase - 0.75f);
            float cmdVel = (phase < 0.25f || phase >= 0.75f) ? 4f * amp / period : -4f * amp / period;
            m_Servo.SetCommand(cmd, cmdVel);
            yield return new WaitForFixedUpdate();
            Log(sb, t, cmd);

            // second cycle: record joint position when cmd crosses zero
            if (t >= period - 1e-4f)
            {
                if (prevCmd < 0f && cmd >= 0f) risingCross = m_Arm.jointPosition[0];
                if (prevCmd > 0f && cmd <= 0f) fallingCross = m_Arm.jointPosition[0];
            }
            prevCmd = cmd;
        }

        File.WriteAllText(Path.Combine(OutputDir(), "hysteresis.csv"), sb.ToString());

        Assert.IsFalse(float.IsNaN(risingCross), "no rising zero crossing recorded");
        Assert.IsFalse(float.IsNaN(fallingCross), "no falling zero crossing recorded");
        float width = fallingCross - risingCross;
        // Expected: ~ backlash gap + 2 * (friction drag) / Kp, loose bounds.
        Assert.Greater(width, 0.3f * Gap, $"hysteresis width {width} too small");
        Assert.Less(width, 6f * Gap, $"hysteresis width {width} too large");
    }

    /// <summary>
    /// Constant-velocity sweep through the bottom of the pendulum. The load
    /// torque changes sign at theta = 0, so the transmission deflection must
    /// traverse the gap (sign flip of delta) even though the command never
    /// reverses. This is the effect a command-side model cannot reproduce.
    /// </summary>
    [UnityTest]
    public IEnumerator GravityCrossing_DeflectionTraversesGap()
    {
        Physics.gravity = new Vector3(0f, -9.81f, 0f);
        CreatePendulum();
        var sb = new StringBuilder(Header);

        const float start = -0.6f, end = 0.6f, vel = 0.15f;
        int settleSteps = Mathf.RoundToInt(3f / 0.02f);
        for (int i = 0; i < settleSteps; i++)
        {
            m_Servo.SetCommand(start, 0f);
            yield return new WaitForFixedUpdate();
        }

        int steps = Mathf.RoundToInt((end - start) / vel / 0.02f);
        float deltaBefore = 0f, deltaAfter = 0f;
        int nBefore = 0, nAfter = 0;
        for (int i = 0; i < steps; i++)
        {
            float t = i * 0.02f;
            float cmd = start + vel * t;
            m_Servo.SetCommand(cmd, vel);
            yield return new WaitForFixedUpdate();
            Log(sb, t, cmd);

            if (cmd < start + 0.25f * (end - start)) { deltaBefore += m_Servo.TransmissionDeflection; nBefore++; }
            if (cmd > start + 0.75f * (end - start)) { deltaAfter += m_Servo.TransmissionDeflection; nAfter++; }
        }
        deltaBefore /= nBefore;
        deltaAfter /= nAfter;

        File.WriteAllText(Path.Combine(OutputDir(), "gravity_crossing.csv"), sb.ToString());

        float b = 0.5f * Gap;
        Assert.Less(deltaBefore, -0.3f * b,
            $"before crossing the load should run ahead (delta<0), got {deltaBefore}");
        Assert.Greater(deltaAfter, 0.3f * b,
            $"after crossing the motor should pull the load (delta>0), got {deltaAfter}");
    }

    /// <summary>
    /// Constant-velocity tracking without gravity or backlash. The steady
    /// motor-side tracking error e = cmd - thetaM satisfies Kp*e = tau_f(w),
    /// so the friction-velocity curve can be recovered and compared with the
    /// configured Stribeck curve.
    /// </summary>
    [UnityTest]
    public IEnumerator FrictionCurve_MatchesStribeckModel()
    {
        Physics.gravity = Vector3.zero;
        CreatePendulum();
        m_Servo.backlashWidth = 0f;
        m_Servo.transmissionStiffness = KtransFriction;
        var sb = new StringBuilder("omega,tauMeasured,tauModel\n");

        var detail = new StringBuilder("w,t,cmd,thetaM,omegaM,thetaL\n");
        float[] speeds = { 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f };
        var measured = new float[speeds.Length];
        var expected = new float[speeds.Length];
        for (int s = 0; s < speeds.Length; s++)
        {
            float w = speeds[s];
            // ramp from the current motor position; no re-centering jump
            float basePos = m_Servo.MotorPosition;
            int steps = Mathf.RoundToInt(5f / 0.02f);
            int tail = Mathf.RoundToInt(1f / 0.02f);
            float errSum = 0f;
            int errN = 0;
            for (int i = 0; i < steps; i++)
            {
                float t = i * 0.02f;
                float cmd = basePos + w * t;
                m_Servo.SetCommand(cmd, w);
                yield return new WaitForFixedUpdate();
                detail.Append(w).Append(',');
                AppendRow(detail, t, cmd, m_Servo.MotorPosition, m_Servo.MotorVelocity, m_Arm.jointPosition[0]);
                if (i >= steps - tail)
                {
                    // Rotor equation at steady state (dwm/dt = 0):
                    //   Jm*dwm/dt = tau_servo - tau_f - tau_transmission
                    //   => tau_f = Kp*e + Kd*(w - wm) - tau_transmission
                    // The transmission term is not zero even here: the load is
                    // dragged at a constant speed, and one step of that travel
                    // is exactly what the dead-zone argument is stale by, so
                    // the rotor carries a residual of order K*w*dt. Dropping
                    // the term inflated the recovered friction by 44 % at
                    // w = 2 rad/s (see docs/Servo-Model-Validation.md).
                    errSum += Kp * (cmd - m_Servo.MotorPosition)
                            + Kd * (w - m_Servo.MotorVelocity)
                            - m_Servo.TransmissionTorque;
                    errN++;
                }
            }
            measured[s] = errSum / errN;
            float x = w / Ws;
            expected[s] = TauC + (TauS - TauC) * Mathf.Exp(-x * x) + Sigma * w;
            AppendRow(sb, w, measured[s], expected[s]);
        }

        File.WriteAllText(Path.Combine(OutputDir(), "friction_curve.csv"), sb.ToString());
        File.WriteAllText(Path.Combine(OutputDir(), "friction_curve_detail.csv"), detail.ToString());

        for (int s = 0; s < speeds.Length; s++)
        {
            Assert.That(measured[s], Is.EqualTo(expected[s]).Within(0.35f * expected[s] + 0.01f),
                $"friction at w={speeds[s]}: measured {measured[s]}, model {expected[s]}");
        }
    }

    /// <summary>
    /// Slow upward sweep against gravity, run twice.
    ///
    /// The rotor cannot slide smoothly below the Stribeck knee: it sticks
    /// (Karnopp parks it at exactly zero), the compliance in front of it winds
    /// up until the torque passes the breakaway value τ_s, the rotor jumps,
    /// and it sticks again. How long that survives is decided by the servo
    /// damping: the validated Kd = 0.5 makes the rotor overdamped
    /// (ζ = Kd/(2·√(Kp·Jm)) = 1.25), so it stick-slips only while breaking
    /// away from rest and then slides. Drop Kd and the limit cycle sustains
    /// over the whole sweep — the textbook dependence, and the reason a
    /// well-damped real servo does not judder.
    ///
    /// The judder is bounded by the two compliances the rotor can wind up
    /// against: (τ_s−τ_c)/(Kp+K) if both springs hold it, up to
    /// (τ_s−τ_c)/K_series with K_series = 1/(1/Kp + 1/K) if they act in series.
    /// </summary>
    [UnityTest]
    public IEnumerator StickSlip_JudderDependsOnServoDamping()
    {
        float lower = (TauS - TauC) / (Kp + Ktrans);
        float upper = (TauS - TauC) / (1f / (1f / Kp + 1f / Ktrans));

        // Both sweeps run at the same speed so that only the damping differs.
        const float vel = 0.07f;   // below the Stribeck knee (omega_s = 0.1)

        // --- lightly damped: a sustained stick-slip limit cycle -------------
        yield return Sweep(0.02f, vel, 8f, "stick_slip.csv");
        int slipsLow = m_SlipsLate;
        float judderLow = m_Judder;
        Assert.Greater(slipsLow, 30,
            $"a lightly damped servo should keep stick-slipping, saw {slipsLow} breakaways");
        Assert.That(judderLow, Is.InRange(0.5f * lower, 2f * upper),
            $"judder {judderLow} rad outside the compliance bounds [{lower}, {upper}]");

        // --- validated damping: the limit cycle is suppressed ---------------
        yield return Sweep(Kd, vel, 8f, "stick_slip_damped.csv");
        Assert.Less(m_SlipsLate, 5,
            $"an overdamped servo (Kd={Kd}, zeta={Kd / (2f * Mathf.Sqrt(Kp * Jm))}) should slide "
            + $"smoothly at the same speed, saw {m_SlipsLate} breakaways");
        Assert.Less(m_Judder, judderLow,
            $"damping should shrink the judder: {m_Judder} rad vs {judderLow} rad undamped");
    }

    /// <summary>
    /// Constant-velocity upward sweep from rest, logged to a CSV. Reports the
    /// breakaway count over the whole run and over the part after the first
    /// 3 s, plus the largest peak-to-peak tracking error inside a 0.8 s window
    /// (short enough that the steadily growing gravity deflection does not
    /// swamp the judder).
    /// </summary>
    int m_Slips, m_SlipsLate;
    float m_Judder;

    IEnumerator Sweep(float servoDamping, float vel, float duration, string csvName)
    {
        Physics.gravity = new Vector3(0f, -9.81f, 0f);
        CreatePendulum();
        m_Servo.servoDamping = servoDamping;
        var sb = new StringBuilder(Header);

        for (int i = 0; i < Mathf.RoundToInt(3f / 0.02f); i++)
        {
            m_Servo.SetCommand(0f, 0f);
            yield return new WaitForFixedUpdate();
        }

        int steps = Mathf.RoundToInt(duration / 0.02f);
        int late = Mathf.Min(steps / 2, Mathf.RoundToInt(3f / 0.02f));
        m_Slips = 0;
        m_SlipsLate = 0;
        bool wasStuck = false;
        var err = new System.Collections.Generic.List<float>();
        for (int i = 0; i < steps; i++)
        {
            float t = i * 0.02f;
            float cmd = vel * t;
            m_Servo.SetCommand(cmd, vel);
            yield return new WaitForFixedUpdate();
            Log(sb, t, cmd);

            bool isStuck = m_Servo.MotorVelocity == 0f;
            if (wasStuck && !isStuck)
            {
                m_Slips++;
                if (i >= late) m_SlipsLate++;
            }
            wasStuck = isStuck;
            if (i >= late) err.Add(cmd - m_Arm.jointPosition[0]);
        }

        File.WriteAllText(Path.Combine(OutputDir(), csvName), sb.ToString());

        m_Judder = 0f;
        const int win = 40; // 0.8 s
        for (int i = 0; i + win < err.Count; i++)
        {
            float lo = float.MaxValue, hi = float.MinValue;
            for (int j = i; j < i + win; j++)
            {
                lo = Mathf.Min(lo, err[j]);
                hi = Mathf.Max(hi, err[j]);
            }
            m_Judder = Mathf.Max(m_Judder, hi - lo);
        }
    }

    /// <summary>
    /// Holding a position against gravity: the joint must settle without
    /// stick-slip chatter and with a bounded error (friction + gap + PD sag).
    /// </summary>
    [UnityTest]
    public IEnumerator StictionHold_NoChatter()
    {
        Physics.gravity = new Vector3(0f, -9.81f, 0f);
        CreatePendulum();
        var sb = new StringBuilder(Header);

        const float cmd = 0.25f;
        int steps = Mathf.RoundToInt(8f / 0.02f);
        int tail = Mathf.RoundToInt(2f / 0.02f);
        // Chatter is measured from the joint POSITION, not from jointVelocity:
        // while an xDrive is driving a joint, jointVelocity reports a floor of
        // ~0.08 rad/s that has nothing to do with the joint's motion. A plain
        // drive with no servo model reports the same 0.08 while its position
        // is bit-identical from step to step (ServoEnvironmentProbe, probe 8).
        float maxVel = 0f;
        float prevTheta = m_Arm.jointPosition[0];
        for (int i = 0; i < steps; i++)
        {
            m_Servo.SetCommand(cmd, 0f);
            yield return new WaitForFixedUpdate();
            Log(sb, i * 0.02f, cmd);
            float theta = m_Arm.jointPosition[0];
            if (i >= steps - tail)
                maxVel = Mathf.Max(maxVel, Mathf.Abs((theta - prevTheta) / Time.fixedDeltaTime));
            prevTheta = theta;
        }

        File.WriteAllText(Path.Combine(OutputDir(), "stiction_hold.csv"), sb.ToString());

        Assert.Less(maxVel, 0.01f, "joint chatters while holding position");
        float err = Mathf.Abs(cmd - m_Arm.jointPosition[0]);
        Assert.Less(err, 0.05f, $"hold error too large: {err}");
    }
}
