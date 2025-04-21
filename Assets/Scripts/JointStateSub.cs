using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSub : MonoBehaviour
{
    public ArticulationBody[] articulationBodies;
    public string[] jointName;
    public string topicName = "/joint_states";
    public int jointLength = 19;
    private List<string> jointNameList;
    private ROSConnection ros;

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

    void Callback(JointStateMsg msg)
    {
        int index;
        for (int i = 0; i < msg.name.Length; i++)
        {
            index = jointNameList.IndexOf(msg.name[i]);
            if (index != -1)
            {
                ArticulationDrive aDrive = articulationBodies[index].xDrive;
                if (i < msg.position.Length)
                    aDrive.target = Mathf.Rad2Deg * (float)msg.position[i];
                if (i < msg.velocity.Length)
                    aDrive.targetVelocity = Mathf.Rad2Deg * (float)msg.velocity[i];
                float effort = float.NaN;
                if (i < msg.effort.Length)
                    effort = (float) msg.effort[i];
                articulationBodies[index].xDrive = aDrive;
            }
        }
    }
}
