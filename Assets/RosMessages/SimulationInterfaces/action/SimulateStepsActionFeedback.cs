using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.SimulationInterfaces
{
    public class SimulateStepsActionFeedback : ActionFeedback<SimulateStepsFeedback>
    {
        public const string k_RosMessageName = "simulation_interfaces/SimulateStepsActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public SimulateStepsActionFeedback() : base()
        {
            this.feedback = new SimulateStepsFeedback();
        }

        public SimulateStepsActionFeedback(HeaderMsg header, GoalStatusMsg status, SimulateStepsFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static SimulateStepsActionFeedback Deserialize(MessageDeserializer deserializer) => new SimulateStepsActionFeedback(deserializer);

        SimulateStepsActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = SimulateStepsFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
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
