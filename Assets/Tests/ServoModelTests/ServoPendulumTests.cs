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
    const float Ktrans = 400f;  // transmission stiffness [N*m/rad]
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
                    // full servo torque balance: Kp*e + Kd*(w - wm) = tau_f
                    errSum += Kp * (cmd - m_Servo.MotorPosition) + Kd * (w - m_Servo.MotorVelocity);
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
        float maxVel = 0f;
        for (int i = 0; i < steps; i++)
        {
            m_Servo.SetCommand(cmd, 0f);
            yield return new WaitForFixedUpdate();
            Log(sb, i * 0.02f, cmd);
            if (i >= steps - tail)
                maxVel = Mathf.Max(maxVel, Mathf.Abs(m_Arm.jointVelocity[0]));
        }

        File.WriteAllText(Path.Combine(OutputDir(), "stiction_hold.csv"), sb.ToString());

        Assert.Less(maxVel, 0.01f, "joint chatters while holding position");
        float err = Mathf.Abs(cmd - m_Arm.jointPosition[0]);
        Assert.Less(err, 0.05f, $"hold error too large: {err}");
    }
}
