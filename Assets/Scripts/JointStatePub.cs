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
        ros.RegisterPublisher<JointStateMsg>(topicName, 15);

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
            effort[i] = articulationBodies[i].driveForce[0];
        }

        // Update pre-allocated message (no new allocations)
        _jointMsg.header.stamp.sec = timestamp.Seconds;
        _jointMsg.header.stamp.nanosec = timestamp.NanoSeconds;
        // Note: position, velocity, effort arrays are already referenced in _jointMsg

        ros.Publish(topicName, _jointMsg);
    }
}
