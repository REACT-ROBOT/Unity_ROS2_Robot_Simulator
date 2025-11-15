using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Tf2;

using Unity.Robotics.Core;
using Unity.Robotics.UrdfImporter;

public class GroundTruthPub : MonoBehaviour
{
    public GameObject targetObject;
    public string topicName = "/ground_truth";
    public string tfTopicName = "/tf";
    public string worldFrameId = "world";
    private ROSConnection ros;

    float time;

    public string frameId = "";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName, 15);
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName, 15);
    }

    void FixedUpdate()
    {
        UrdfLink link = targetObject.GetComponent<UrdfLink>();
        frameId = link.name;

        time += Time.deltaTime;
        if (time<0.02f) return;  // 50Hz update rate for better tf performance
        time = 0.0f;
        
        // Use ROS time (Unix epoch time)
        var unixEpoch = new System.DateTime(1970, 1, 1, 0, 0, 0, System.DateTimeKind.Utc);
        var currentTime = System.DateTime.UtcNow;
        var timeSpan = currentTime - unixEpoch;
        
        int sec = (int)timeSpan.TotalSeconds;
        uint nanosec = (uint)((timeSpan.TotalSeconds - sec) * 1e9);

        // Convert Unity coordinates (left-handed) to ROS coordinates (right-handed)
        // Unity: X=right, Y=up, Z=forward
        // ROS: X=forward, Y=left, Z=up
        Vector3 unityPos = targetObject.transform.position;
        Quaternion unityRot = targetObject.transform.rotation;
        
        // Unity to ROS coordinate transformation
        Vector3 rosPosition = new Vector3(unityPos.z, -unityPos.x, unityPos.y);
        Quaternion rosRotation = new Quaternion(-unityRot.z, unityRot.x, -unityRot.y, unityRot.w);

        PoseStampedMsg pose_msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg
                {
                    sec = (int)sec,
                    nanosec = (uint)nanosec,
                },
            },
            pose = new PoseMsg
            {
                position = new PointMsg
                {
                    x = rosPosition.x,
                    y = rosPosition.y,
                    z = rosPosition.z,
                },
                orientation = new QuaternionMsg
                {
                    x = rosRotation.x,
                    y = rosRotation.y,
                    z = rosRotation.z,
                    w = rosRotation.w,
                }
            }
        };

        ros.Publish(topicName, pose_msg);

        // Publish TF message
        TransformStampedMsg transformStamped = new TransformStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = worldFrameId,
                stamp = new TimeMsg
                {
                    sec = (int)sec,
                    nanosec = (uint)nanosec,
                },
            },
            child_frame_id = frameId,
            transform = new TransformMsg
            {
                translation = new Vector3Msg
                {
                    x = rosPosition.x,
                    y = rosPosition.y,
                    z = rosPosition.z,
                },
                rotation = new QuaternionMsg
                {
                    x = rosRotation.x,
                    y = rosRotation.y,
                    z = rosRotation.z,
                    w = rosRotation.w,
                }
            }
        };

        TFMessageMsg tf_msg = new TFMessageMsg
        {
            transforms = new TransformStampedMsg[] { transformStamped }
        };

        ros.Publish(tfTopicName, tf_msg);
    }
}
