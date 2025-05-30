//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.SimulationInterfaces
{
    [Serializable]
    public class EntityStateMsg : Message
    {
        public const string k_RosMessageName = "simulation_interfaces/EntityState";
        public override string RosMessageName => k_RosMessageName;

        //  Entity current pose, twist and acceleration
        public Std.HeaderMsg header;
        //  Reference frame and timestamp for pose and twist. Empty frame defaults to world.
        public Geometry.PoseMsg pose;
        //  Pose in reference frame, ground truth.
        public Geometry.TwistMsg twist;
        //  Ground truth linear and angular velocities
        //  observed in the frame specified by header.frame_id
        //  See https://github.com/ros2/common_interfaces/pull/240 for conventions.
        public Geometry.AccelMsg acceleration;
        //  Linear and angular acceleration ground truth, following the same convention.

        public EntityStateMsg()
        {
            this.header = new Std.HeaderMsg();
            this.pose = new Geometry.PoseMsg();
            this.twist = new Geometry.TwistMsg();
            this.acceleration = new Geometry.AccelMsg();
        }

        public EntityStateMsg(Std.HeaderMsg header, Geometry.PoseMsg pose, Geometry.TwistMsg twist, Geometry.AccelMsg acceleration)
        {
            this.header = header;
            this.pose = pose;
            this.twist = twist;
            this.acceleration = acceleration;
        }

        public static EntityStateMsg Deserialize(MessageDeserializer deserializer) => new EntityStateMsg(deserializer);

        private EntityStateMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.pose = Geometry.PoseMsg.Deserialize(deserializer);
            this.twist = Geometry.TwistMsg.Deserialize(deserializer);
            this.acceleration = Geometry.AccelMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.pose);
            serializer.Write(this.twist);
            serializer.Write(this.acceleration);
        }

        public override string ToString()
        {
            return "EntityStateMsg: " +
            "\nheader: " + header.ToString() +
            "\npose: " + pose.ToString() +
            "\ntwist: " + twist.ToString() +
            "\nacceleration: " + acceleration.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
