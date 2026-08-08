using System.Collections;
using System.Globalization;
using System.IO;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Diagnostic probes for the two environment-dependent assumptions inside
/// ServoJointModel:
///   1. the effective unit of a rotational xDrive stiffness (N*m/rad vs the
///      pi/180-scaled player behaviour), and
///   2. whether ArticulationBody.jointVelocity of a statically held joint
///      carries the pre-solve gravity bias.
/// Both are measured on a PLAIN drive (no ServoJointModel) so the result is a
/// property of the engine in this run mode, not of the model.
/// Results are printed to the log and written to TestOutput/env_probe.txt.
/// </summary>
public class ServoEnvironmentProbe
{
    const float Mass = 0.2f;
    const float Length = 0.2f;
    const float Inertia = 2e-3f;
    const float G = 9.81f;
    // Gravity torque coefficient: tau_g = -MgL * sin(theta)
    const float MgL = Mass * G * Length;

    GameObject m_Root;
    ArticulationBody m_Arm;
    Vector3 m_SavedGravity;
    float m_SavedTimeScale;
    float m_SavedMaxDelta;
    static readonly StringBuilder s_Report = new StringBuilder();

    [SetUp]
    public void SetUp()
    {
        m_SavedGravity = Physics.gravity;
        m_SavedTimeScale = Time.timeScale;
        m_SavedMaxDelta = Time.maximumDeltaTime;
        Time.fixedDeltaTime = 0.02f;
        Time.timeScale = 10f;
        Time.maximumDeltaTime = 1f;
        Physics.gravity = new Vector3(0f, -G, 0f);
    }

    [TearDown]
    public void TearDown()
    {
        if (m_Root != null)
            Object.Destroy(m_Root);
        Physics.gravity = m_SavedGravity;
        Time.timeScale = m_SavedTimeScale;
        Time.maximumDeltaTime = m_SavedMaxDelta;
        Directory.CreateDirectory(OutputDir());
        File.WriteAllText(Path.Combine(OutputDir(), "env_probe.txt"), s_Report.ToString());
    }

    static string OutputDir()
    {
        return Path.Combine(Application.dataPath, "..", "TestOutput");
    }

    static void Say(string line)
    {
        s_Report.Append(line).Append('\n');
        Debug.Log("[ENVPROBE] " + line);
    }

    static string F(float v)
    {
        return v.ToString("G7", CultureInfo.InvariantCulture);
    }

    /// <summary>Plain revolute arm with a position drive, no servo model.</summary>
    void CreateArm(float stiffness, float damping, float targetDeg)
    {
        m_Root = new GameObject("probe_base");
        var baseBody = m_Root.AddComponent<ArticulationBody>();
        baseBody.immovable = true;

        var arm = new GameObject("probe_arm");
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
        m_Arm.inertiaTensor = new Vector3(Inertia, Inertia, Inertia);
        m_Arm.inertiaTensorRotation = Quaternion.identity;
        m_Arm.jointFriction = 0f;
        m_Arm.linearDamping = 0f;
        m_Arm.angularDamping = 0f;
        m_Arm.useGravity = true;
        m_Arm.sleepThreshold = 0f;
        baseBody.sleepThreshold = 0f;

        var d = m_Arm.xDrive;
        d.stiffness = stiffness;
        d.damping = damping;
        d.forceLimit = 1e4f;
        d.target = targetDeg;
        d.targetVelocity = 0f;
        m_Arm.xDrive = d;
    }

    /// <summary>
    /// Probe 1: static sag of a plain position drive under a known gravity
    /// torque tells us what a nominal stiffness S actually holds with.
    ///   S_eff = MgL*sin(theta_eq) / (theta_target - theta_eq)
    /// ratio S_eff/S == 1     -> editor convention (N*m/rad, no compensation)
    /// ratio S_eff/S == pi/180 -> player convention (needs Rad2Deg compensation)
    /// </summary>
    [UnityTest]
    public IEnumerator Probe1_DriveStiffnessUnit()
    {
        Say("=== environment ===");
        Say($"isEditor={Application.isEditor} isBatchMode={Application.isBatchMode} "
            + $"platform={Application.platform} unity={Application.unityVersion}");
#if UNITY_EDITOR
        Say("UNITY_EDITOR is DEFINED in this assembly");
#else
        Say("UNITY_EDITOR is NOT defined in this assembly");
#endif
        Say($"fixedDeltaTime={F(Time.fixedDeltaTime)} timeScale={F(Time.timeScale)} "
            + $"simulationMode={Physics.simulationMode} solverIterations={Physics.defaultSolverIterations}");

        const float S = 100f;
        const float targetDeg = 90f;
        float targetRad = targetDeg * Mathf.Deg2Rad;
        CreateArm(S, 5f, targetDeg);

        // Let it settle (long, damping is modest on purpose so nothing rings).
        int steps = Mathf.RoundToInt(10f / 0.02f);
        for (int i = 0; i < steps; i++)
            yield return new WaitForFixedUpdate();

        float theta = m_Arm.jointPosition[0];
        float sag = targetRad - theta;
        float tauG = MgL * Mathf.Sin(theta);
        float sEff = Mathf.Abs(sag) > 1e-9f ? tauG / sag : float.NaN;

        Say("=== probe 1: xDrive stiffness unit ===");
        Say($"nominal stiffness S = {F(S)}, target = {F(targetDeg)} deg = {F(targetRad)} rad");
        Say($"settled jointPosition = {F(theta)} rad ({F(theta * Mathf.Rad2Deg)} deg)");
        Say($"sag = {F(sag)} rad, gravity torque at rest = {F(tauG)} N*m");
        Say($"=> effective stiffness = {F(sEff)} N*m/rad, ratio S_eff/S = {F(sEff / S)}");
        Say($"   (1.0 => editor units; {F(Mathf.Deg2Rad)} => player units)");
        Say($"driveForce = {F(m_Arm.driveForce[0])}");
    }

    /// <summary>
    /// Probe 2: does jointVelocity of a joint held static under gravity report
    /// the pre-solve gravity step instead of ~0? Compare it against a position
    /// finite difference over the same fixed step.
    /// </summary>
    [UnityTest]
    public IEnumerator Probe2_JointVelocityBias()
    {
        // Stiff enough to hold hard regardless of which unit convention is in
        // effect, so the joint is genuinely static in both cases.
        const float S = 20000f;
        const float targetDeg = 45f;
        CreateArm(S, 200f, targetDeg);

        int settle = Mathf.RoundToInt(10f / 0.02f);
        for (int i = 0; i < settle; i++)
            yield return new WaitForFixedUpdate();

        float prev = m_Arm.jointPosition[0];
        float maxReported = 0f, maxFd = 0f, sumReported = 0f, sumFd = 0f;
        const int n = 50;
        for (int i = 0; i < n; i++)
        {
            yield return new WaitForFixedUpdate();
            float now = m_Arm.jointPosition[0];
            float fd = (now - prev) / Time.fixedDeltaTime;
            float rep = m_Arm.jointVelocity[0];
            prev = now;
            maxReported = Mathf.Max(maxReported, Mathf.Abs(rep));
            maxFd = Mathf.Max(maxFd, Mathf.Abs(fd));
            sumReported += rep;
            sumFd += fd;
        }

        float theta = m_Arm.jointPosition[0];
        float tauG = MgL * Mathf.Sin(theta);
        // Effective rotational inertia about the joint axis.
        float j = Inertia + Mass * Length * Length;
        float predictedBias = tauG / j * Time.fixedDeltaTime;

        Say("=== probe 2: jointVelocity bias of a statically held joint ===");
        Say($"settled jointPosition = {F(theta)} rad ({F(theta * Mathf.Rad2Deg)} deg)");
        Say($"mean jointVelocity   = {F(sumReported / n)}  (max |.| = {F(maxReported)})");
        Say($"mean finite-diff vel = {F(sumFd / n)}  (max |.| = {F(maxFd)})");
        Say($"predicted gravity-step bias tau_g/J*dt = {F(predictedBias)} rad/s "
            + $"(tau_g={F(tauG)}, J={F(j)})");
        Say("   reported ~ predicted bias while FD ~ 0  => jointVelocity is biased (player-like)");
        Say("   reported ~ FD ~ 0                       => jointVelocity is clean (editor-like)");
    }

    /// <summary>
    /// Probe 3: unit of xDrive.targetVelocity for a rotational drive.
    /// Pure velocity drive (stiffness = 0) without gravity settles at exactly
    /// the commanded velocity, so the settled joint speed reveals the unit:
    ///   omega == V        -> targetVelocity is consumed as rad/s (NOT converted)
    ///   omega == V*pi/180 -> targetVelocity is in deg/s (converted by the engine)
    /// The rise time also gives the effective damping unit: tau = J / D_eff.
    /// </summary>
    [UnityTest]
    public IEnumerator Probe3_TargetVelocityUnit()
    {
        Physics.gravity = Vector3.zero;
        const float V = 1.0f;   // nominal commanded velocity
        const float D = 2.0f;   // nominal damping [N*m/(rad/s)]
        CreateArm(0f, D, 0f);
        var d = m_Arm.xDrive;
        d.targetVelocity = V;
        m_Arm.xDrive = d;

        float prev = m_Arm.jointPosition[0];
        float firstFd = float.NaN;
        for (int i = 0; i < Mathf.RoundToInt(5f / 0.02f); i++)
        {
            yield return new WaitForFixedUpdate();
            float now = m_Arm.jointPosition[0];
            if (i == 0) firstFd = (now - prev) / Time.fixedDeltaTime;
            prev = now;
        }

        // Settled speed over the last 25 steps.
        float p0 = m_Arm.jointPosition[0];
        for (int i = 0; i < 25; i++)
            yield return new WaitForFixedUpdate();
        float omega = (m_Arm.jointPosition[0] - p0) / (25f * Time.fixedDeltaTime);
        float j = Inertia + Mass * Length * Length;

        Say("=== probe 3: xDrive targetVelocity / damping unit ===");
        Say($"commanded targetVelocity = {F(V)} (nominal), damping = {F(D)}");
        Say($"settled joint speed = {F(omega)} rad/s   (reported jointVelocity = {F(m_Arm.jointVelocity[0])})");
        Say($"   {F(V)} => targetVelocity consumed as rad/s; {F(V * Mathf.Deg2Rad)} => deg/s (converted)");
        Say($"first-step speed = {F(firstFd)} rad/s; J/D = {F(j / D)} s vs dt = {F(Time.fixedDeltaTime)}");
        Say($"driveForce at speed = {F(m_Arm.driveForce[0])}");
    }

    /// <summary>
    /// Probe 4: is the jointVelocity seen by FixedUpdate current or one step
    /// stale WHILE MOVING? Probe 2 only covers the static case, but the
    /// transmission stability at K = 400 N*m/rad hinges on there being no extra
    /// step of delay in the load velocity.
    /// A freely accelerating pendulum is integrated semi-implicitly, so
    /// theta_n - theta_{n-1} == dt * omega_n. Comparing the reported velocity
    /// against the backward difference of the SAME step therefore separates
    /// "current" from "one step behind".
    /// </summary>
    [UnityTest]
    public IEnumerator Probe4_JointVelocityFreshness()
    {
        // theta = 0 is the hanging equilibrium for -Y gravity, so tilt gravity
        // into -Z: the torque about the X twist axis is then maximal at theta=0
        // and the arm accelerates away from rest immediately.
        Physics.gravity = new Vector3(0f, 0f, -G);
        CreateArm(0f, 0f, 0f);       // no drive at all: free swing under gravity
        yield return new WaitForFixedUpdate();

        var sb = new StringBuilder("step,theta,omegaReported,backwardFD\n");
        float prev = m_Arm.jointPosition[0];
        float sumErrCurrent = 0f, sumErrStale = 0f, sumRatio = 0f;
        float prevReported = 0f;
        int n = 0;
        for (int i = 0; i < 40; i++)
        {
            yield return new WaitForFixedUpdate();
            float now = m_Arm.jointPosition[0];
            float bfd = (now - prev) / Time.fixedDeltaTime;
            float rep = m_Arm.jointVelocity[0];
            sb.Append(i).Append(',');
            AppendRow(sb, now, rep, bfd);
            if (i > 0)
            {
                sumErrCurrent += Mathf.Abs(rep - bfd);       // reported == this step's velocity
                sumErrStale += Mathf.Abs(prevReported - bfd); // reported == previous step's velocity
                sumRatio += Mathf.Abs(bfd) > 1e-9f ? rep / bfd : 0f;
                n++;
            }
            prev = now;
            prevReported = rep;
        }
        Directory.CreateDirectory(OutputDir());
        File.WriteAllText(Path.Combine(OutputDir(), "env_probe_freefall.csv"), sb.ToString());

        Say("=== probe 4: jointVelocity freshness while moving (free fall) ===");
        Say($"mean |reported_n - backwardFD_n|     = {F(sumErrCurrent / n)}   (0 => current)");
        Say($"mean |reported_(n-1) - backwardFD_n| = {F(sumErrStale / n)}   (0 => one step stale)");
        Say($"mean reported_n / backwardFD_n       = {F(sumRatio / n)}   (+1 => same sign; -1 => SIGN INVERTED)");
        Say($"final theta = {F(m_Arm.jointPosition[0])} rad, omega = {F(m_Arm.jointVelocity[0])} rad/s");
    }

    /// <summary>
    /// Probe 5: where is the discrete stability limit of the transmission
    /// stiffness at 50 Hz? Runs the StictionHold scenario (hold 0.25 rad
    /// against gravity) through the real ServoJointModel for a range of
    /// transmission stiffnesses and reports the settled chatter velocity.
    /// The pendulum tests are configured at K = 400 N*m/rad.
    /// </summary>
    [UnityTest]
    public IEnumerator Probe5_TransmissionStiffnessSweep()
    {
        float[] stiffnesses = { 20f, 50f, 100f, 200f, 300f, 400f, 800f };
        Say("=== probe 5: transmission stiffness stability sweep (StictionHold rig) ===");
        Say("max|omega| over the last 2 s of an 8 s hold; > 1 means the loop has diverged");
        Say("     | " + string.Join(" | ", System.Array.ConvertAll(stiffnesses, k => $"K={F(k),6}")));

        var row = new StringBuilder("hold |");
        foreach (float k in stiffnesses)
        {
            Physics.gravity = new Vector3(0f, -G, 0f);
            var root = new GameObject("sweep_base");
            var baseBody = root.AddComponent<ArticulationBody>();
            baseBody.immovable = true;
            var arm = new GameObject("sweep_arm");
            arm.transform.SetParent(root.transform, false);
            var body = arm.AddComponent<ArticulationBody>();
            body.jointType = ArticulationJointType.RevoluteJoint;
            body.twistLock = ArticulationDofLock.FreeMotion;
            body.anchorPosition = Vector3.zero;
            body.anchorRotation = Quaternion.identity;
            body.mass = Mass;
            body.automaticCenterOfMass = false;
            body.centerOfMass = new Vector3(0f, -Length, 0f);
            body.automaticInertiaTensor = false;
            body.inertiaTensor = new Vector3(Inertia, Inertia, Inertia);
            body.inertiaTensorRotation = Quaternion.identity;
            body.jointFriction = 0f;
            body.linearDamping = 0f;
            body.angularDamping = 0f;
            body.useGravity = true;

            // Same servo parameters as ServoPendulumTests.
            var servo = arm.AddComponent<ServoJointModel>();
            servo.servoStiffness = 20f;
            servo.servoDamping = 0.5f;
            servo.motorTorqueLimit = 2f;
            servo.motorInertia = 2e-3f;
            servo.staticFriction = 0.15f;
            servo.dynamicFriction = 0.08f;
            servo.stribeckVelocity = 0.1f;
            servo.viscousFriction = 0.005f;
            servo.backlashWidth = 2f * 0.0087f;
            servo.transmissionStiffness = k;
            servo.transmissionDamping = 0.5f;

            const float cmd = 0.25f;
            int steps = Mathf.RoundToInt(8f / 0.02f);
            int tail = Mathf.RoundToInt(2f / 0.02f);
            float maxVel = 0f;
            for (int i = 0; i < steps; i++)
            {
                servo.SetCommand(cmd, 0f);
                yield return new WaitForFixedUpdate();
                if (i >= steps - tail)
                    maxVel = Mathf.Max(maxVel, Mathf.Abs(body.jointVelocity[0]));
            }
            row.Append($" {F(maxVel),7} |");
            Object.Destroy(root);
            yield return new WaitForFixedUpdate();
        }
        Say(row.ToString());
    }

    /// <summary>
    /// Probe 6: probe 4 found jointVelocity exact on a joint with NO drive,
    /// yet the servo tests read a velocity of the opposite sign on a joint the
    /// transmission drive is moving. Isolate the drive as the only variable:
    /// a PLAIN position drive (no ServoJointModel) whose target is ramped at a
    /// constant rate, with and without gravity.
    /// </summary>
    [UnityTest]
    public IEnumerator Probe6_JointVelocitySignUnderDrive()
    {
        Say("=== probe 6: jointVelocity sign while a plain xDrive moves the joint ===");
        Say("case            | mean reported | mean backwardFD | ratio");

        for (int c = 0; c < 2; c++)
        {
            bool withGravity = c == 0;
            Physics.gravity = withGravity ? new Vector3(0f, -G, 0f) : Vector3.zero;
            CreateArm(200f, 0.5f, -35f);

            // Settle at the start angle.
            for (int i = 0; i < Mathf.RoundToInt(3f / 0.02f); i++)
                yield return new WaitForFixedUpdate();

            const float rateDeg = 0.15f * Mathf.Rad2Deg; // 0.15 rad/s, as in GravityCrossing
            float prev = m_Arm.jointPosition[0];
            float sumRep = 0f, sumFd = 0f;
            int n = Mathf.RoundToInt(8f / 0.02f);
            for (int i = 0; i < n; i++)
            {
                var d = m_Arm.xDrive;
                d.target = -35f + rateDeg * (i * 0.02f);
                d.targetVelocity = rateDeg;
                m_Arm.xDrive = d;
                yield return new WaitForFixedUpdate();
                float now = m_Arm.jointPosition[0];
                sumFd += (now - prev) / Time.fixedDeltaTime;
                sumRep += m_Arm.jointVelocity[0];
                prev = now;
            }
            float mr = sumRep / n, mf = sumFd / n;
            Say($"{(withGravity ? "with gravity   " : "zero gravity   ")} | {F(mr),13} | {F(mf),15} | {F(mr / mf)}");

            Object.Destroy(m_Root);
            m_Root = null;
            yield return new WaitForFixedUpdate();
        }
        Say("   ratio +1 => consistent; -1 => jointVelocity sign inverted under an active drive");
    }

    /// <summary>
    /// Probe 7: the GravityCrossing scenario across transmission stiffnesses.
    /// The dead-zone argument is stale by one step of load travel, and that
    /// error reaches the joint as K*sech^2(delta/b)*omega*dt, so the flank the
    /// model perceives should only come out right once K*omega*dt is small
    /// against the gravity torque the transmission has to carry.
    /// Before the bottom the load is falling with gravity and must run AHEAD
    /// of the rotor (delta &lt; 0); after it the rotor must pull (delta &gt; 0).
    /// </summary>
    [UnityTest]
    public IEnumerator Probe7_GravityCrossingVsStiffness()
    {
        float[] stiffnesses = { 5f, 10f, 20f, 50f, 100f, 200f, 400f };
        const float b = 0.0087f;
        Say("=== probe 7: perceived contact flank vs transmission stiffness ===");
        Say($"backlash half width b = {F(b)} rad; want deltaBefore < {F(-0.3f * b)} and deltaAfter > {F(0.3f * b)}");
        Say("K [N*m/rad] | K*omega*dt | deltaBefore | deltaAfter  | flank");

        foreach (float k in stiffnesses)
        {
            Physics.gravity = new Vector3(0f, -G, 0f);
            var root = new GameObject("gc_base");
            var baseBody = root.AddComponent<ArticulationBody>();
            baseBody.immovable = true;
            var arm = new GameObject("gc_arm");
            arm.transform.SetParent(root.transform, false);
            var body = arm.AddComponent<ArticulationBody>();
            body.jointType = ArticulationJointType.RevoluteJoint;
            body.twistLock = ArticulationDofLock.FreeMotion;
            body.anchorPosition = Vector3.zero;
            body.anchorRotation = Quaternion.identity;
            body.mass = Mass;
            body.automaticCenterOfMass = false;
            body.centerOfMass = new Vector3(0f, -Length, 0f);
            body.automaticInertiaTensor = false;
            body.inertiaTensor = new Vector3(Inertia, Inertia, Inertia);
            body.inertiaTensorRotation = Quaternion.identity;
            body.jointFriction = 0f;
            body.linearDamping = 0f;
            body.angularDamping = 0f;
            body.useGravity = true;

            var servo = arm.AddComponent<ServoJointModel>();
            servo.servoStiffness = 20f;
            servo.servoDamping = 0.5f;
            servo.motorTorqueLimit = 2f;
            servo.motorInertia = 2e-3f;
            servo.staticFriction = 0.15f;
            servo.dynamicFriction = 0.08f;
            servo.stribeckVelocity = 0.1f;
            servo.viscousFriction = 0.005f;
            servo.backlashWidth = 2f * b;
            servo.transmissionStiffness = k;
            servo.transmissionDamping = 0.5f;

            const float start = -0.6f, end = 0.6f, vel = 0.15f;
            for (int i = 0; i < Mathf.RoundToInt(3f / 0.02f); i++)
            {
                servo.SetCommand(start, 0f);
                yield return new WaitForFixedUpdate();
            }
            int steps = Mathf.RoundToInt((end - start) / vel / 0.02f);
            float dBefore = 0f, dAfter = 0f;
            int nB = 0, nA = 0;
            for (int i = 0; i < steps; i++)
            {
                float cmd = start + vel * (i * 0.02f);
                servo.SetCommand(cmd, vel);
                yield return new WaitForFixedUpdate();
                if (cmd < start + 0.25f * (end - start)) { dBefore += servo.TransmissionDeflection; nB++; }
                if (cmd > start + 0.75f * (end - start)) { dAfter += servo.TransmissionDeflection; nA++; }
            }
            dBefore /= nB; dAfter /= nA;
            bool ok = dBefore < -0.3f * b && dAfter > 0.3f * b;
            Say($"K = {F(k),8} | {F(k * vel * 0.02f),10} | {F(dBefore),11} | {F(dAfter),11} | {(ok ? "correct" : "wrong")}");

            Object.Destroy(root);
            yield return new WaitForFixedUpdate();
        }
    }

    /// <summary>
    /// Builds the ServoPendulumTests rig (immovable base + one revolute arm)
    /// and returns the servo, so the probes can reuse the exact validation
    /// setup. Pass servo = false for a plain drive with no model at all.
    /// </summary>
    ServoJointModel BuildPendulum(float k, bool servo, out ArticulationBody body,
                                  float backlash = 2f * 0.0087f, float damping = 0.5f,
                                  float tauS = 0.15f)
    {
        m_Root = new GameObject("hold_base");
        var baseBody = m_Root.AddComponent<ArticulationBody>();
        baseBody.immovable = true;
        var arm = new GameObject("hold_arm");
        arm.transform.SetParent(m_Root.transform, false);
        body = arm.AddComponent<ArticulationBody>();
        body.jointType = ArticulationJointType.RevoluteJoint;
        body.twistLock = ArticulationDofLock.FreeMotion;
        body.anchorPosition = Vector3.zero;
        body.anchorRotation = Quaternion.identity;
        body.mass = Mass;
        body.automaticCenterOfMass = false;
        body.centerOfMass = new Vector3(0f, -Length, 0f);
        body.automaticInertiaTensor = false;
        body.inertiaTensor = new Vector3(Inertia, Inertia, Inertia);
        body.inertiaTensorRotation = Quaternion.identity;
        body.jointFriction = 0f;
        body.linearDamping = 0f;
        body.angularDamping = 0f;
        body.useGravity = true;
        body.sleepThreshold = 0f;
        baseBody.sleepThreshold = 0f;

        if (!servo)
        {
            var d = body.xDrive;
            d.stiffness = k;
            d.damping = damping;
            d.forceLimit = 1e3f;
            d.target = 0.25f * Mathf.Rad2Deg;
            d.targetVelocity = 0f;
            body.xDrive = d;
            return null;
        }

        var s = arm.AddComponent<ServoJointModel>();
        s.servoStiffness = 20f;
        s.servoDamping = 0.5f;
        s.motorTorqueLimit = 2f;
        s.motorInertia = 2e-3f;
        s.staticFriction = tauS;
        s.dynamicFriction = tauS > 0f ? 0.08f : 0f;
        s.stribeckVelocity = 0.1f;
        s.viscousFriction = 0.005f;
        s.backlashWidth = backlash;
        s.transmissionStiffness = k;
        s.transmissionDamping = damping;
        return s;
    }

    /// <summary>
    /// Probe 8: StictionHold_NoChatter asserts on ArticulationBody.jointVelocity,
    /// which probe 6 showed is not a usable readback while an xDrive is driving
    /// the joint. Measure the same hold with BOTH the reported velocity and a
    /// position finite difference (plus the actual peak-to-peak travel), and
    /// include a plain drive with no servo model as the engine's noise floor.
    /// </summary>
    [UnityTest]
    public IEnumerator Probe8_HoldChatterReportedVsTrue()
    {
        Say("=== probe 8: is the hold chatter real, or a jointVelocity artefact? ===");
        Say("8 s hold at 0.25 rad against gravity; statistics over the last 2 s");
        Say("case                  | max|reported| | max|posFD| | p-p travel [rad]");

        var cases = new[]
        {
            new { label = "servo K=2   gap=1deg", k = 2f,   servo = true,  gap = 2f * 0.0087f, tauS = 0.15f },
            new { label = "servo K=50  gap=1deg", k = 50f,  servo = true,  gap = 2f * 0.0087f, tauS = 0.15f },
            new { label = "servo K=50  gap=0   ", k = 50f,  servo = true,  gap = 0f,           tauS = 0.15f },
            new { label = "servo K=50  no fric ", k = 50f,  servo = true,  gap = 2f * 0.0087f, tauS = 0f },
            new { label = "servo K=400 gap=1deg", k = 400f, servo = true,  gap = 2f * 0.0087f, tauS = 0.15f },
            new { label = "plain drive K=50    ", k = 50f,  servo = false, gap = 0f,           tauS = 0f },
        };

        foreach (var c in cases)
        {
            Physics.gravity = new Vector3(0f, -G, 0f);
            var servo = BuildPendulum(c.k, c.servo, out ArticulationBody body, c.gap, 0.5f, c.tauS);

            int steps = Mathf.RoundToInt(8f / 0.02f);
            int tail = Mathf.RoundToInt(2f / 0.02f);
            float maxRep = 0f, maxFd = 0f, lo = float.MaxValue, hi = float.MinValue;
            float prev = body.jointPosition[0];
            for (int i = 0; i < steps; i++)
            {
                if (servo != null) servo.SetCommand(0.25f, 0f);
                yield return new WaitForFixedUpdate();
                float now = body.jointPosition[0];
                if (i >= steps - tail)
                {
                    maxRep = Mathf.Max(maxRep, Mathf.Abs(body.jointVelocity[0]));
                    maxFd = Mathf.Max(maxFd, Mathf.Abs((now - prev) / Time.fixedDeltaTime));
                    lo = Mathf.Min(lo, now); hi = Mathf.Max(hi, now);
                }
                prev = now;
            }
            Say($"{c.label} | {F(maxRep),13} | {F(maxFd),10} | {F(hi - lo)}");
            Object.Destroy(m_Root);
            m_Root = null;
            yield return new WaitForFixedUpdate();
        }
        Say("   a large reported value next to a tiny posFD/travel means the");
        Say("   test is reading the broken API, not a real oscillation");
    }

    /// <summary>
    /// Probe 9: FrictionCurve_MatchesStribeckModel assumes the motor-side
    /// torque balance Kp*e + Kd*(w - wm) is pure friction, i.e. that the
    /// transmission carries nothing while the load is dragged at a constant
    /// speed with no gravity. Break the balance into its terms to see whether
    /// that assumption still holds.
    /// </summary>
    [UnityTest]
    public IEnumerator Probe9_FrictionBalanceTerms()
    {
        const float Kp = 20f, Kd = 0.5f, TauS = 0.15f, TauC = 0.08f, Ws = 0.1f, Sigma = 0.005f;
        Say("=== probe 9: what the friction-curve balance actually contains ===");
        Say("zero gravity, no backlash, constant-velocity ramp; mean over the last 1 s");
        Say("K    | w    | Kp*e+Kd*dw | model friction | tauTrans carried | balance-friction");

        foreach (float k in new[] { 2f, 50f, 400f })
        foreach (float w in new[] { 0.1f, 1.0f })
        {
            Physics.gravity = Vector3.zero;
            var servo = BuildPendulum(k, true, out ArticulationBody body, 0f);

            float basePos = servo.MotorPosition;
            int steps = Mathf.RoundToInt(5f / 0.02f);
            int tail = Mathf.RoundToInt(1f / 0.02f);
            float sumBal = 0f, sumTrans = 0f;
            int n = 0;
            for (int i = 0; i < steps; i++)
            {
                float cmd = basePos + w * (i * 0.02f);
                servo.SetCommand(cmd, w);
                yield return new WaitForFixedUpdate();
                if (i >= steps - tail)
                {
                    sumBal += Kp * (cmd - servo.MotorPosition) + Kd * (w - servo.MotorVelocity);
                    sumTrans += servo.TransmissionTorque;
                    n++;
                }
            }
            float bal = sumBal / n, trans = sumTrans / n;
            float x = w / Ws;
            float fric = TauC + (TauS - TauC) * Mathf.Exp(-x * x) + Sigma * w;
            Say($"{F(k),4} | {F(w),4} | {F(bal),10} | {F(fric),14} | {F(trans),16} | {F(bal - fric)}");

            Object.Destroy(m_Root);
            m_Root = null;
            yield return new WaitForFixedUpdate();
        }
    }

    /// <summary>
    /// Probe 10: stick-slip needs the motor to be dragged below the Stribeck
    /// knee through a series compliance while a load torque holds it back.
    /// Sweep the pendulum slowly upwards under gravity at several speeds and
    /// report how often the rotor is stuck (Karnopp) together with the judder
    /// amplitude, which should come out near (tau_s - tau_c)/K_series with
    /// K_series = 1/(1/Kp + 1/K).
    /// </summary>
    [UnityTest]
    public IEnumerator Probe10_StickSlipConditions()
    {
        Say("=== probe 10: where does the servo stick-slip? ===");
        Say("slow upward sweep under gravity, K = 50, omega_s = 0.1 rad/s");
        Say("Kd    | sweep w | stuck (mid) | slips (mid) | judder p-p [rad] | expected (tau_s-tau_c)/K_series");

        float kSeries = 1f / (1f / 20f + 1f / 50f);
        float expected = (0.15f - 0.08f) / kSeries;
        // Sweep speed AND servo damping: Kd resists the breakaway jump, so it
        // is the parameter that decides whether stick-slip is a startup
        // transient or a sustained limit cycle.
        foreach (float kd in new[] { 0.5f, 0.1f, 0.02f })
        foreach (float w in new[] { 0.02f, 0.05f, 0.07f, 0.1f, 0.15f })
        {
            Physics.gravity = new Vector3(0f, -G, 0f);
            var servo = BuildPendulum(50f, true, out ArticulationBody body);
            servo.servoDamping = kd;

            for (int i = 0; i < Mathf.RoundToInt(3f / 0.02f); i++)
            {
                servo.SetCommand(0f, 0f);
                yield return new WaitForFixedUpdate();
            }

            int steps = Mathf.RoundToInt(Mathf.Min(20f, 0.6f / w) / 0.02f);
            // Skip the first 3 s: breaking away from rest always produces one
            // stick-slip burst. What matters is whether it SUSTAINS.
            int skip = Mathf.Min(steps / 2, Mathf.RoundToInt(3f / 0.02f));
            int stuck = 0, slips = 0;
            bool wasStuck = false;
            var err = new System.Collections.Generic.List<float>();
            for (int i = 0; i < steps; i++)
            {
                float cmd = w * (i * 0.02f);
                servo.SetCommand(cmd, w);
                yield return new WaitForFixedUpdate();
                bool isStuck = servo.MotorVelocity == 0f;
                if (i >= skip)
                {
                    if (isStuck) stuck++;
                    if (wasStuck && !isStuck) slips++;
                    err.Add(cmd - body.jointPosition[0]);
                }
                wasStuck = isStuck;
            }
            // Local peak-to-peak with the gravity trend removed, so the
            // judder is not swamped by the rising steady deflection.
            float pp = 0f;
            const int win = 40; // 0.8 s
            for (int i = 0; i + win < err.Count; i++)
            {
                float mn = float.MaxValue, mx = float.MinValue;
                for (int j = i; j < i + win; j++) { mn = Mathf.Min(mn, err[j]); mx = Mathf.Max(mx, err[j]); }
                pp = Mathf.Max(pp, mx - mn);
            }
            Say($"{F(kd),5} | {F(w),7} | {stuck,5}/{err.Count,-5} | {slips,11} | {F(pp),16} | {F(expected)}");
            Object.Destroy(m_Root);
            m_Root = null;
            yield return new WaitForFixedUpdate();
        }
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
}
