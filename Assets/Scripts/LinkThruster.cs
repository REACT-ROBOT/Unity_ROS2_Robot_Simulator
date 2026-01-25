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

    private void OnCommand(Float32Msg msg)
    {
        command = msg.data;
    }
}
