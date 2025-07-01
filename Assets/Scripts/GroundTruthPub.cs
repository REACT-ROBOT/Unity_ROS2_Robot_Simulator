using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;

public class GroundTruthPub : MonoBehaviour
{
    public GameObject targetObject;
    public string topicName = "/ground_truth";
    private ROSConnection ros;

    float time;

    public string frameId = "";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName, 15);
    }

    void FixedUpdate()
    {
        time += Time.deltaTime;
        if (time<0.05f) return;
        time = 0.0f;
        var timestamp = new TimeStamp(Clock.Now);

        PoseStampedMsg pose_msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                },
            },
            pose = new PoseMsg
            {
                position = new PointMsg
                {
                    x = targetObject.transform.position.x,
                    y = targetObject.transform.position.y,
                    z = targetObject.transform.position.z,
                },
                orientation = new QuaternionMsg
                {
                    x = targetObject.transform.rotation.x,
                    y = targetObject.transform.rotation.y,
                    z = targetObject.transform.rotation.z,
                    w = targetObject.transform.rotation.w,
                }
            }
        };

        ros.Publish(topicName, pose_msg);
    }
}
