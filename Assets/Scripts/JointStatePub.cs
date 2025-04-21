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

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName, 15);

        position = new double[jointName.Length];
        velocity = new double[jointName.Length];
        effort = new double[jointName.Length];
    }

    void FixedUpdate()
    {
        time += Time.deltaTime;
        if (time<0.05f) return;
        time = 0.0f;
        var timestamp = new TimeStamp(Clock.Now);

        for (int i = 0; i < articulationBodies.Length; i++)
        {
            ArticulationDrive xDrive = this.articulationBodies[i].xDrive;
            position[i] = articulationBodies[i].jointPosition[0]/Mathf.Rad2Deg;
            velocity[i] = articulationBodies[i].jointVelocity[0]/Mathf.Rad2Deg;
            effort[i] = articulationBodies[i].driveForce[0]/Mathf.Rad2Deg;
        }

        JointStateMsg joint_msg = new JointStateMsg{
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg{
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                },
            },
            name = jointName,
            position = position,
            velocity = velocity,
            effort = effort,
        };

        ros.Publish(topicName, joint_msg);
    }
}
