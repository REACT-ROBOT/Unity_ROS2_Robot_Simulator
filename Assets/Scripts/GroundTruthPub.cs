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

    // Cached DateTime for Unix epoch (avoid allocation every frame)
    private static readonly System.DateTime UnixEpoch = new System.DateTime(1970, 1, 1, 0, 0, 0, System.DateTimeKind.Utc);

    // Pre-allocated messages to avoid GC allocations
    private PoseStampedMsg _poseMsg;
    private TFMessageMsg _tfMsg;
    private TransformStampedMsg[] _tfTransforms;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName, 15);
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName, 15);

        // Pre-allocate messages once to avoid GC allocations every frame
        _poseMsg = new PoseStampedMsg
        {
            header = new HeaderMsg { stamp = new TimeMsg() },
            pose = new PoseMsg
            {
                position = new PointMsg(),
                orientation = new QuaternionMsg()
            }
        };

        _tfTransforms = new TransformStampedMsg[1];
        _tfTransforms[0] = new TransformStampedMsg
        {
            header = new HeaderMsg { stamp = new TimeMsg() },
            transform = new TransformMsg
            {
                translation = new Vector3Msg(),
                rotation = new QuaternionMsg()
            }
        };

        _tfMsg = new TFMessageMsg { transforms = _tfTransforms };
    }

    void FixedUpdate()
    {
        UrdfLink link = targetObject.GetComponent<UrdfLink>();
        frameId = link.name;

        time += Time.deltaTime;
        if (time<0.02f) return;  // 50Hz update rate for better tf performance
        time = 0.0f;

        // Use ROS time (Unix epoch time) - use cached UnixEpoch to avoid allocation
        var currentTime = System.DateTime.UtcNow;
        var timeSpan = currentTime - UnixEpoch;

        int sec = (int)timeSpan.TotalSeconds;
        uint nanosec = (uint)((timeSpan.TotalSeconds - sec) * 1e9);

        // Convert Unity coordinates (left-handed) to ROS coordinates (right-handed)
        // Unity: X=right, Y=up, Z=forward
        // ROS: X=forward, Y=left, Z=up
        Vector3 unityPos = targetObject.transform.position;
        Quaternion unityRot = targetObject.transform.rotation;

        // Unity to ROS coordinate transformation
        float rosX = unityPos.z;
        float rosY = -unityPos.x;
        float rosZ = unityPos.y;
        float rotX = -unityRot.z;
        float rotY = unityRot.x;
        float rotZ = -unityRot.y;
        float rotW = unityRot.w;

        // Update pre-allocated PoseStampedMsg (no new allocations)
        _poseMsg.header.frame_id = frameId;
        _poseMsg.header.stamp.sec = sec;
        _poseMsg.header.stamp.nanosec = nanosec;
        _poseMsg.pose.position.x = rosX;
        _poseMsg.pose.position.y = rosY;
        _poseMsg.pose.position.z = rosZ;
        _poseMsg.pose.orientation.x = rotX;
        _poseMsg.pose.orientation.y = rotY;
        _poseMsg.pose.orientation.z = rotZ;
        _poseMsg.pose.orientation.w = rotW;

        ros.Publish(topicName, _poseMsg);

        // Update pre-allocated TFMessageMsg (no new allocations)
        _tfTransforms[0].header.frame_id = worldFrameId;
        _tfTransforms[0].header.stamp.sec = sec;
        _tfTransforms[0].header.stamp.nanosec = nanosec;
        _tfTransforms[0].child_frame_id = frameId;
        _tfTransforms[0].transform.translation.x = rosX;
        _tfTransforms[0].transform.translation.y = rosY;
        _tfTransforms[0].transform.translation.z = rosZ;
        _tfTransforms[0].transform.rotation.x = rotX;
        _tfTransforms[0].transform.rotation.y = rotY;
        _tfTransforms[0].transform.rotation.z = rotZ;
        _tfTransforms[0].transform.rotation.w = rotW;

        ros.Publish(tfTopicName, _tfMsg);
    }
}
