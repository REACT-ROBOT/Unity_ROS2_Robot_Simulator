using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;

public class JointStatePub : MonoBehaviour
{
    public ArticulationBody[] articulationBodies;
    public string topicName = "/joint_states";
    public int jointLength = 19;
    // Feedback publish rate [Hz]. Robot controllers poll their motor feedback at
    // their own control cycle (BL26: 30 Hz) and get the LATEST joint state, so
    // this bounds how stale that feedback can be. Values at or above the physics
    // rate (1 / fixedDeltaTime) publish every physics step.
    public float publishRateHz = 30f;
    private ROSConnection ros;

    float time;

    public string frameId = "";
    public string[] jointName = new string[] {};
    public double[] position = new double[] {};
    public double[] velocity = new double[] {};
    public double[] effort = new double[] {};

    // Pre-allocated message to avoid GC allocations
    private JointStateMsg _jointMsg;

    // 速度フィードバックは jointVelocity ではなく位置の差分から作る。
    // PhysX の jointVelocity はドライブ+接触摩擦の平衡で静止中も定常残差
    // (~0.01 rad/s) を返し、実位置の変化 (~1e-4 rad/s) と桁違いに食い違う。
    // これをホイールオドメトリが積分すると停止中でも heading がドリフトする。
    // 位置差分なら実機エンコーダと同じ算出で、姿勢差分由来の IMU とも整合する。
    private double[] _lastPosition;
    private bool _feedbackBaselineValid;
    // Real time since the last publish, for the velocity difference above --
    // kept separate from the schedule accumulator (which carries a remainder).
    private float _elapsedSincePublish;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // 同名トピックへの二重登録を避ける。デスポーンしても publisher の登録は
        // 残る (ROSConnection に解除 API が無い) ため、再スポーン時にそのまま
        // 登録し直すと "registered twice!" の警告でログが埋まり、本物の異常が
        // 埋もれてしまう。既存の登録はそのまま使える。
        RosTopicState topicState = ros.GetTopic(topicName);
        if (topicState == null || !topicState.IsPublisher)
        {
            ros.RegisterPublisher<JointStateMsg>(topicName, 15);
        }

        position = new double[jointName.Length];
        velocity = new double[jointName.Length];
        effort = new double[jointName.Length];
        _lastPosition = new double[jointName.Length];

        // Pre-allocate message once to avoid GC allocations every frame
        _jointMsg = new JointStateMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg()
            },
            name = jointName,
            position = position,
            velocity = velocity,
            effort = effort
        };
    }

    /// <summary>
    /// 速度差分の基準位置を無効化する。リセットやテレポートで関節位置が
    /// 不連続に飛んだ直後に、偽の速度スパイクを publish しないために呼ぶ。
    /// </summary>
    public void ResetFeedback()
    {
        _feedbackBaselineValid = false;
    }

    void FixedUpdate()
    {
        time += Time.deltaTime;
        _elapsedSincePublish += Time.deltaTime;
        float interval = (publishRateHz > 0f) ? (1f / publishRateHz) : 0.05f;
        if (time < interval) return;
        // dt is the REAL elapsed time since the last publish -- the position
        // difference below must be divided by it. It is NOT the schedule
        // accumulator: that one carries a remainder credit, and dividing by it
        // scales the velocity feedback by real/dt (up to ~2x off), which the
        // robot's wheel odometry then integrates into a distance error.
        float dt = _elapsedSincePublish;
        _elapsedSincePublish = 0f;
        // Carry the remainder so the AVERAGE rate holds even when the interval
        // is not a multiple of the physics step (50 Hz steps + 30 Hz target ->
        // alternating 20/40 ms). Clamp the carry so a hitch cannot burst.
        time -= interval;
        if (time > interval) time = interval;
        var timestamp = new TimeStamp(Clock.Now);

        for (int i = 0; i < articulationBodies.Length; i++)
        {
            double pos = articulationBodies[i].jointPosition[0];
            position[i] = pos;
            velocity[i] = _feedbackBaselineValid ? (pos - _lastPosition[i]) / dt : 0.0;
            _lastPosition[i] = pos;
            // driveForce は xDrive が出した力、jointForce は effort 指令で
            // 直接与えた一般化力。各関節でどちらか一方しか使われないので、
            // 和を取ればモードを知らずに済む。
            effort[i] = articulationBodies[i].driveForce[0]
                + articulationBodies[i].jointForce[0];
        }
        _feedbackBaselineValid = true;

        // Update pre-allocated message (no new allocations)
        _jointMsg.header.stamp.sec = timestamp.Seconds;
        _jointMsg.header.stamp.nanosec = timestamp.NanoSeconds;
        // Note: position, velocity, effort arrays are already referenced in _jointMsg

        ros.Publish(topicName, _jointMsg);
    }
}
