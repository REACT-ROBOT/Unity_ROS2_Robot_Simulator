using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class LinkThruster : MonoBehaviour
{
    public ArticulationBody targetBody;
    public string topicName = "";
    public float maxForce = 100f;
    public Vector3 localDirection = Vector3.forward;
    public float command = 0f;
    public bool clampCommand = true;

    private ROSConnection ros;
    private bool unsubscribed;

    void Awake()
    {
        if (targetBody == null)
        {
            targetBody = GetComponentInParent<ArticulationBody>();
        }
    }

    void Start()
    {
        if (!string.IsNullOrEmpty(topicName))
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<Float32Msg>(topicName, OnCommand);
        }
    }

    void FixedUpdate()
    {
        if (targetBody == null)
        {
            return;
        }

        float throttle = command;
        if (clampCommand)
        {
            throttle = Mathf.Clamp(throttle, -1f, 1f);
        }

        Vector3 direction = transform.TransformDirection(localDirection.normalized);
        Vector3 force = direction * (throttle * maxForce);
        if (force.sqrMagnitude > 0.0001f)
        {
            targetBody.AddForceAtPosition(force, transform.position);
        }
    }

    /// <summary>
    /// このインスタンスを ROS の受信経路から切り離す。デスポーン時に必ず呼ぶこと。
    /// </summary>
    /// <remarks>
    /// JointStateSub.DetachFromRos と同じ理由・同じ作り。破棄済みインスタンスの
    /// コールバックが購読リストに残ると、そこで例外が出た時点で同じトピックの
    /// 後続コールバックが呼ばれなくなる。
    /// </remarks>
    public void DetachFromRos()
    {
        if (unsubscribed)
        {
            return;
        }
        unsubscribed = true;
        if (ros != null && !string.IsNullOrEmpty(topicName))
        {
            ros.Unsubscribe(topicName);
        }
    }

    void OnDestroy()
    {
        DetachFromRos();
    }

    private void OnCommand(Float32Msg msg)
    {
        if (unsubscribed)
        {
            return;
        }
        command = msg.data;
    }
}
