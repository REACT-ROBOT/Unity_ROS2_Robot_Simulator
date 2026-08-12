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
    private ROSConnection ros;

    float time;

    public string frameId = "";
    public string[] jointName = new string[] {};
    public double[] position = new double[] {};
    public double[] velocity = new double[] {};
    public double[] effort = new double[] {};

    // Pre-allocated message to avoid GC allocations
    private JointStateMsg _jointMsg;

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

    void FixedUpdate()
    {
        time += Time.deltaTime;
        if (time<0.05f) return;
        time = 0.0f;
        var timestamp = new TimeStamp(Clock.Now);

        for (int i = 0; i < articulationBodies.Length; i++)
        {
            position[i] = articulationBodies[i].jointPosition[0];
            velocity[i] = articulationBodies[i].jointVelocity[0];
            // driveForce は xDrive が出した力、jointForce は effort 指令で
            // 直接与えた一般化力。各関節でどちらか一方しか使われないので、
            // 和を取ればモードを知らずに済む。
            effort[i] = articulationBodies[i].driveForce[0]
                + articulationBodies[i].jointForce[0];
        }

        // Update pre-allocated message (no new allocations)
        _jointMsg.header.stamp.sec = timestamp.Seconds;
        _jointMsg.header.stamp.nanosec = timestamp.NanoSeconds;
        // Note: position, velocity, effort arrays are already referenced in _jointMsg

        ros.Publish(topicName, _jointMsg);
    }
}
