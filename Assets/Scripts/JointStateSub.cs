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
    public string[] jointName;
    public string topicName = "/joint_states";
    public int jointLength = 19;
    private List<string> jointNameList;
    private ROSConnection ros;
    private bool unsubscribed;

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
