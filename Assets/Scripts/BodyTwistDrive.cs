using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

/// <summary>
/// 車体速度 (geometry_msgs/Twist) を受けて、車体を力とトルクで追従させる駆動要素。
/// オムニホイールのように「多数の車輪リンクと床の摩擦」で走らせる代わりに、
/// 車体そのものへ直接力を加えて指令速度に一致させる。
/// </summary>
/// <remarks>
/// 何のためにあるか:
/// オムニホイールは 1 輪あたりフリーローラ 8 個 + ハウジング 2 個で、4 輪ぶんだと
/// 40 リンクになる。これがソルバのコストの大半を占め、ロボット台数を増やすと
/// 物理が実時間に追いつかなくなる。駆動を車体への直接加力に置き換えると
/// 駆動系のリンクが丸ごと不要になり、1 台あたりのコストが大きく下がる。
///
/// なぜ速度を代入せず力で追従させるか:
/// ArticulationBody.linearVelocity へ直接代入すると最も軽く正確になるが、
/// 毎ステップ上書きするため他機に押されなくなる。接触のある競技では
/// 押し合いが成立しないと困るので、力で追従させる。外力を受けている間は
/// 指令速度からずれるが、毎ステップ誤差を詰め直すので外力が消えれば戻る。
///
/// 制御則:
/// 各 FixedUpdate で「指令速度 - 現在速度」を 1 ステップで詰めるのに必要な
/// 加速度を求め、最大加速度でクランプしてから、質量 (慣性) を掛けて力
/// (トルク) にする。クランプに掛からない範囲では 1 ステップで目標へ到達する
/// ので、定常偏差は出ない。クランプが実質的な加減速の制限になり、
/// omni_wheel_controller が持っていた加速度制限の役割を引き継ぐ。
///
/// 何を制御しないか:
/// 鉛直方向 (Unity の Y) とロール・ピッチには触らない。重力と法線力、
/// 衝突による転倒をそのまま物理に任せるため。制御するのは水平 2 方向と
/// ヨーだけ。
/// </remarks>
public class BodyTwistDrive : MonoBehaviour
{
    public ArticulationBody targetBody;
    public string topicName = "";

    [Header("指令の上限 (ROS 座標系・車体基準)")]
    public float maxLinearVelocity = 3.6f;       // [m/s]
    public float maxAngularVelocity = 6.0f;      // [rad/s]
    public float maxLinearAcceleration = 4.0f;   // [m/s^2]
    public float maxAngularAcceleration = 8.0f;  // [rad/s^2]

    /// <summary>
    /// 指令が途切れてから停止するまでのシミュレーション時間 [s]。
    /// 0 以下で無効 (最後の指令を保持し続ける)。
    /// </summary>
    /// <remarks>
    /// omni_wheel_controller の cmd_vel_timeout と同じ役割。操縦端末との
    /// 通信が切れた機体が指令速度のまま走り続けるのを防ぐ。
    /// </remarks>
    public float commandTimeout = 0.5f;

    /// <summary>
    /// 追従の内訳をログへ出す間隔 [シミュレーション秒]。0 以下で出さない。
    /// </summary>
    /// <remarks>
    /// 指令どおりに動かないときに、原因が「指令の向きの取り違え」なのか
    /// 「力が出ていない」のか「外力に負けている」のかを切り分けるためのもの。
    /// 既定は 0 (無効) なので、通常の実行では何も出ない。
    /// </remarks>
    public float debugInterval = 0f;

    private float m_LastDebugTime = float.NegativeInfinity;

    // 指令 (ROS 座標系・車体基準)。ROS スレッドから書かれ FixedUpdate から読まれる。
    private volatile float m_CmdVx;
    private volatile float m_CmdVy;
    private volatile float m_CmdWz;
    private float m_LastCommandTime = float.NegativeInfinity;
    private bool m_HasCommand;

    private ROSConnection m_Ros;
    private bool m_Unsubscribed;

    // 起動時に一度だけ求める。アーティキュレーション全体の質量とヨー慣性。
    private float m_TotalMass = 1f;
    private float m_YawInertia = 1f;
    // 重心を毎ステップ求め直すために、連結の構成を控えておく。
    private ArticulationBody[] m_Bodies = System.Array.Empty<ArticulationBody>();

    void Awake()
    {
        if (targetBody == null)
        {
            targetBody = GetComponentInParent<ArticulationBody>();
        }
    }

    void Start()
    {
        RecomputeInertialProperties();

        if (!string.IsNullOrEmpty(topicName))
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.Subscribe<TwistMsg>(topicName, OnCommand);
        }
    }

    /// <summary>
    /// 加力の換算に使う質量とヨー慣性を、アーティキュレーション全体から求める。
    /// </summary>
    /// <remarks>
    /// 根の ArticulationBody へ力を加えると連結全体が動くので、換算に要るのは
    /// 全体の質量であって根リンク単体の質量ではない。ヨー慣性も同様に、
    /// 各リンクの「重心まわりの慣性」と「回転軸からの距離による寄与 (m r^2)」を
    /// 足し合わせる。値がずれると 1 ステップで詰めきれず応答が鈍る (過大なら
    /// 行き過ぎる) が、クランプがあるので発散はしない。
    /// </remarks>
    private void RecomputeInertialProperties()
    {
        if (targetBody == null)
        {
            return;
        }

        ArticulationBody[] bodies = targetBody.GetComponentsInChildren<ArticulationBody>(true);
        m_Bodies = bodies;
        float mass = 0f;
        foreach (ArticulationBody body in bodies)
        {
            mass += body.mass;
        }
        if (mass <= 0f)
        {
            return;
        }
        m_TotalMass = mass;

        Vector3 com = CenterOfMass();
        Vector3 up = Vector3.up;
        float inertia = 0f;
        foreach (ArticulationBody body in bodies)
        {
            // 回転軸 (重心を通る鉛直線) からの距離による寄与
            Vector3 offset = body.worldCenterOfMass - com;
            Vector3 radial = Vector3.ProjectOnPlane(offset, up);
            inertia += body.mass * radial.sqrMagnitude;

            // リンク自身の慣性テンソルを鉛直軸へ射影する。慣性テンソルは
            // inertiaTensorRotation が表す主軸系で対角なので、各主軸と鉛直の
            // なす角の余弦の 2 乗で重み付けして足す。
            Quaternion principal = body.transform.rotation * body.inertiaTensorRotation;
            Vector3 tensor = body.inertiaTensor;
            for (int axis = 0; axis < 3; axis++)
            {
                Vector3 dir = principal * (axis == 0 ? Vector3.right
                                         : axis == 1 ? Vector3.up
                                                     : Vector3.forward);
                float c = Vector3.Dot(dir, up);
                inertia += tensor[axis] * c * c;
            }
        }
        m_YawInertia = Mathf.Max(inertia, 1e-4f);
    }

    /// <summary>連結全体の重心 (ワールド座標)。</summary>
    /// <remarks>
    /// 加力はここに掛けないといけない。ArticulationBody.AddForce は「その
    /// リンクの重心」に力を加えるので、根リンク (base_link は質量ほぼ 0 の
    /// 座標系リンク) に対して呼ぶと、連結全体の重心からずれた点に力が
    /// 掛かり、その腕の長さぶんのトルクが出る。摩擦 0 では打ち消す力が
    /// 無いので、機体はまっすぐ進まず回り出す。
    /// </remarks>
    private Vector3 CenterOfMass()
    {
        float mass = 0f;
        Vector3 weighted = Vector3.zero;
        foreach (ArticulationBody body in m_Bodies)
        {
            if (body == null)
            {
                continue;
            }
            mass += body.mass;
            weighted += body.worldCenterOfMass * body.mass;
        }
        return mass > 0f ? weighted / mass : targetBody.worldCenterOfMass;
    }

    /// <summary>連結全体の重心の速度 (ワールド座標)。</summary>
    /// <remarks>
    /// 根リンクの linearVelocity をそのまま使うと、機体が旋回しているときに
    /// 重心まわりの回転ぶん (ω × r) が乗る。制御したいのは重心の並進なので、
    /// 質量で重み付けして平均する。
    /// </remarks>
    private Vector3 CenterOfMassVelocity()
    {
        float mass = 0f;
        Vector3 weighted = Vector3.zero;
        foreach (ArticulationBody body in m_Bodies)
        {
            if (body == null)
            {
                continue;
            }
            mass += body.mass;
            weighted += body.linearVelocity * body.mass;
        }
        return mass > 0f ? weighted / mass : targetBody.linearVelocity;
    }

    void FixedUpdate()
    {
        if (targetBody == null)
        {
            return;
        }

        float vx = 0f, vy = 0f, wz = 0f;
        bool active = m_HasCommand;
        if (active && commandTimeout > 0f &&
            Time.fixedTime - m_LastCommandTime > commandTimeout)
        {
            // 指令切れ。目標 0 にして止めにいく (力は出し続けるので、
            // 惰性で流れずその場に止まる)。
            active = false;
        }
        if (active)
        {
            vx = m_CmdVx;
            vy = m_CmdVy;
            wz = m_CmdWz;
        }
        // 指令が無いとき (未受信・指令切れ) は目標 0 として止めにいく。
        // 何もしないでいると、この駆動を使う機体は床の摩擦を 0 にしてあるため、
        // スポーン時のわずかな初速や接触の弾みで滑り出したまま永久に止まらない。
        // 実測では、指令を出す前の 15 秒で 1.2 m 離して置いた 2 台が
        // 9.9 m まで離れていた。鉛直方向は制御していないので、着地は
        // これまでどおり重力に任せたまま。

        // --- 並進 ---------------------------------------------------------
        // 指令は ROS 座標系・車体基準。Unity 座標へ直してから車体姿勢で回す。
        Vector3 targetLocal = RosToUnityVector(vx, vy, 0f);
        Vector3 targetWorld = targetBody.transform.TransformDirection(targetLocal);
        targetWorld = Vector3.ClampMagnitude(
            Vector3.ProjectOnPlane(targetWorld, Vector3.up), maxLinearVelocity);

        // 鉛直成分は触らない。重力と法線力をそのまま効かせる。
        Vector3 current = Vector3.ProjectOnPlane(CenterOfMassVelocity(), Vector3.up);
        Vector3 accel = (targetWorld - current) / Time.fixedDeltaTime;
        accel = Vector3.ClampMagnitude(accel, maxLinearAcceleration);
        Vector3 force = accel * m_TotalMass;
        targetBody.AddForce(force);

        // AddForce は targetBody 自身の重心に掛かる。根リンク (base_link) は
        // 質量ほぼ 0 の座標系リンクで、連結全体の重心とは離れているため、
        // そのずれが腕の長さとなってトルクが出る。摩擦 0 では打ち消す力が
        // 無いので、放っておくと機体が回り出す (実測: vx=1.0 の指令で
        // +8.8 deg/s の意図しない旋回)。同じだけの逆トルクを足して消す。
        //
        // AddForceAtPosition で重心に直接掛ける手もあるが、ArticulationBody
        // では並進がまったく出なかった (実測 0%) ので、動作が確かな
        // AddForce と AddTorque の組み合わせで等価なことをする。
        Vector3 lever = targetBody.worldCenterOfMass - CenterOfMass();
        if (lever.sqrMagnitude > 1e-10f)
        {
            targetBody.AddTorque(-Vector3.Cross(lever, force));
        }

        if (debugInterval > 0f && Time.fixedTime - m_LastDebugTime >= debugInterval)
        {
            m_LastDebugTime = Time.fixedTime;
            Transform t = targetBody.transform;
            Debug.Log(
                $"[BodyTwistDrive] {topicName}\n" +
                $"  指令(ROS体)   vx={vx:F3} vy={vy:F3} wz={wz:F3}\n" +
                $"  目標(U体)     {targetLocal}\n" +
                $"  目標(Uワールド) {targetWorld}  現在 {current}\n" +
                $"  力 {force} (質量 {m_TotalMass:F2} kg)  " +
                $"腕 {lever} \n" +
                $"  車体軸: fwd={t.forward} right={t.right} up={t.up}\n" +
                $"  重心 {CenterOfMass()}  根重心 {targetBody.worldCenterOfMass}");
        }

        // --- ヨー ---------------------------------------------------------
        // ROS のヨー (+z まわり反時計) は Unity では -y まわり。
        float targetYaw = -Mathf.Clamp(wz, -maxAngularVelocity, maxAngularVelocity);
        float currentYaw = targetBody.angularVelocity.y;
        float angAccel = (targetYaw - currentYaw) / Time.fixedDeltaTime;
        angAccel = Mathf.Clamp(angAccel, -maxAngularAcceleration, maxAngularAcceleration);
        targetBody.AddTorque(new Vector3(0f, angAccel * m_YawInertia, 0f));
    }

    /// <summary>ROS のベクトル (x 前・y 左・z 上) を Unity 座標へ。</summary>
    /// <remarks>SimulationEntityServices の RosToUnityVector と同じ対応。</remarks>
    private static Vector3 RosToUnityVector(float x, float y, float z) =>
        new Vector3(-y, z, x);

    /// <summary>
    /// このインスタンスを ROS の受信経路から切り離す。デスポーン時に必ず呼ぶこと。
    /// </summary>
    /// <remarks>
    /// LinkThruster.DetachFromRos と同じ理由・同じ作り。破棄済みインスタンスの
    /// コールバックが購読リストに残ると、そこで例外が出た時点で同じトピックの
    /// 後続コールバックが呼ばれなくなる。
    /// </remarks>
    public void DetachFromRos()
    {
        if (m_Unsubscribed)
        {
            return;
        }
        m_Unsubscribed = true;
        if (m_Ros != null && !string.IsNullOrEmpty(topicName))
        {
            m_Ros.Unsubscribe(topicName);
        }
    }

    void OnDestroy()
    {
        DetachFromRos();
    }

    private void OnCommand(TwistMsg msg)
    {
        if (m_Unsubscribed)
        {
            return;
        }
        m_CmdVx = (float)msg.linear.x;
        m_CmdVy = (float)msg.linear.y;
        m_CmdWz = (float)msg.angular.z;
        m_LastCommandTime = Time.fixedTime;
        m_HasCommand = true;
    }
}
