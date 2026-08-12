using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSub : MonoBehaviour
{
    public ArticulationBody[] articulationBodies;
    // Optional per-joint servo model (friction/backlash). When present at an
    // index, commands are routed to the model instead of writing xDrive.
    public ServoJointModel[] servoModels;
    // ros2_control の <command_interface name="effort"/> を宣言した関節。
    // true の関節はドライブ (xDrive) ではなく jointForce でトルク指令する。
    // スポーン時 (SimulationControl) に jointName と同順で設定される。null なら全関節従来どおり。
    public bool[] effortModes;
    // effort 指令のクランプ [N·m / N]。URDF の <limit effort> と
    // command_interface の max パラメータの小さい方。
    public float[] effortLimits;
    public string[] jointName;
    public string topicName = "/joint_states";
    public int jointLength = 19;
    private List<string> jointNameList;
    private ROSConnection ros;
    private bool unsubscribed;
    // 最後に指令されたトルク。effort インターフェース流に、新しい指令が来るまで
    // 保持して毎 FixedUpdate 適用し続ける (リセット時はゼロへ)。
    private float[] commandedEfforts;

    // Set Parameters
    public float stiffness = 0F;
    public float damping = 10000000F;
    public float forceLimit = float.MaxValue;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, Callback);

        jointNameList = new List<string>(jointName);
    }

    /// <summary>
    /// このインスタンスを ROS の受信経路から切り離す。デスポーン時に必ず呼ぶこと。
    /// </summary>
    /// <remarks>
    /// 切り離さないと、破棄済みの ArticulationBody を触るコールバックが
    /// ROSConnection (RosTopicState) の購読リストに残り続ける。RosTopicState は
    /// コールバックを List.ForEach で回すため、残骸が NullReferenceException を
    /// 投げた時点で列挙が中断し、その後ろに登録された「再スポーンしたロボットの
    /// コールバック」まで呼ばれなくなる。これがリセット後に指令を受け付けなく
    /// なる原因だった。
    ///
    /// フラグと購読解除の両方を行う。ROSConnection.Unsubscribe() はトピック単位で
    /// コールバックを全消しするうえ、実際に反映されるのはエンドポイントへ
    /// __remove_subscriber が届いた後なので、それまでの間に届くメッセージは
    /// フラグ側で弾く必要がある。
    ///
    /// OnDestroy 任せにしないのは、Unity の Destroy がフレーム終端まで遅延される
    /// ため。デスポーン側から明示的に呼ぶことで順序を確定させる。
    ///
    /// 注意: __remove_subscriber は ROS-TCP-Endpoint 側の対応が要る。本家
    /// (Unity-Technologies) の v0.7.0 は未実装で、受け取ると handle_syscommand の
    /// getattr が AttributeError を投げて TCP 接続ごと落ちる。hijimasa/ROS-TCP-Endpoint
    /// では実装済みで、かつ未知コマンドで接続が落ちないようにしてある。
    /// </remarks>
    public void DetachFromRos()
    {
        if (unsubscribed)
            return;
        unsubscribed = true;
        if (ros != null)
            ros.Unsubscribe(topicName);
    }

    void OnDestroy()
    {
        // 明示的に切り離されないまま破棄された場合の保険。
        DetachFromRos();
    }

    /// <summary>保持中の effort 指令をゼロへ戻す。リセット時に呼ぶ。</summary>
    public void ResetCommands()
    {
        if (commandedEfforts != null)
        {
            for (int i = 0; i < commandedEfforts.Length; i++)
            {
                commandedEfforts[i] = 0f;
            }
        }
    }

    /// <summary>この関節が effort 指令モードか (GUI のスライダ無効化用)。</summary>
    public bool IsEffortJoint(string joint)
    {
        if (effortModes == null || jointNameList == null)
            return false;
        int index = jointNameList.IndexOf(joint);
        return index >= 0 && index < effortModes.Length && effortModes[index];
    }

    void FixedUpdate()
    {
        // effort モードの関節へ、保持中のトルクを毎ステップ適用する。
        // jointForce は縮約座標系の一般化力 [N·m / N] で、xDrive の度単位とは
        // 無関係。リセット (ResetArticulationState) が jointForce をゼロに
        // 書くので、指令が残っている限りここで書き直す。
        if (effortModes == null || commandedEfforts == null || articulationBodies == null)
            return;

        for (int i = 0; i < articulationBodies.Length && i < effortModes.Length; i++)
        {
            if (!effortModes[i])
                continue;
            ArticulationBody body = articulationBodies[i];
            if (body == null || body.dofCount != 1)
                continue;
            var force = body.jointForce;
            force[0] = commandedEfforts[i];
            body.jointForce = force;
        }
    }

    void Callback(JointStateMsg msg)
    {
        // 解除済み / 破棄途中のインスタンスは何もしない。ここで例外を投げると
        // 同じトピックの後続コールバックが呼ばれなくなる (UnsubscribeFromRos の
        // 注記を参照) ため、この Callback は絶対に例外を出さないようにする。
        if (unsubscribed || articulationBodies == null || jointNameList == null)
            return;

        int index;
        for (int i = 0; i < msg.name.Length; i++)
        {
            index = jointNameList.IndexOf(msg.name[i]);
            if (index == -1 || index >= articulationBodies.Length)
                continue;

            // effort モードの関節はトルク指令だけを受け取り、position/velocity は
            // 無視する (ドライブは無効化済みなので書いても意味がない)。
            if (effortModes != null && index < effortModes.Length && effortModes[index])
            {
                if (i < msg.effort.Length)
                {
                    float tau = (float)msg.effort[i];
                    if (effortLimits != null && index < effortLimits.Length
                        && !float.IsInfinity(effortLimits[index]))
                    {
                        tau = Mathf.Clamp(tau, -effortLimits[index], effortLimits[index]);
                    }
                    if (commandedEfforts == null)
                        commandedEfforts = new float[articulationBodies.Length];
                    commandedEfforts[index] = tau;
                }
                continue;
            }

            ServoJointModel servo = (servoModels != null && index < servoModels.Length)
                ? servoModels[index] : null;
            if (servo != null)
            {
                // Servo model works in SI joint space (rad, rad/s).
                float pos = servo.targetPosition;
                float vel = 0f;
                if (i < msg.position.Length)
                    pos = (float)msg.position[i];
                if (i < msg.velocity.Length)
                    vel = (float)msg.velocity[i];
                servo.SetCommand(pos, vel);
                continue;
            }

            ArticulationBody body = articulationBodies[index];
            // Unity の破棄済みオブジェクトは == null が true になる。
            if (body == null)
                continue;

            ArticulationDrive aDrive = body.xDrive;
            if (i < msg.position.Length)
                aDrive.target = Mathf.Rad2Deg * (float)msg.position[i];
            if (i < msg.velocity.Length)
                aDrive.targetVelocity = Mathf.Rad2Deg * (float)msg.velocity[i];
            body.xDrive = aDrive;
        }
    }
}
