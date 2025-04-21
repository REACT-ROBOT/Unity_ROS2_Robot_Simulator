using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.SimulationInterfaces
{
    public class SimulateStepsActionResult : ActionResult<SimulateStepsResult>
    {
        public const string k_RosMessageName = "simulation_interfaces/SimulateStepsActionResult";
        public override string RosMessageName => k_RosMessageName;


        public SimulateStepsActionResult() : base()
        {
            this.result = new SimulateStepsResult();
        }

        public SimulateStepsActionResult(HeaderMsg header, GoalStatusMsg status, SimulateStepsResult result) : base(header, status)
        {
            this.result = result;
        }
        public static SimulateStepsActionResult Deserialize(MessageDeserializer deserializer) => new SimulateStepsActionResult(deserializer);

        SimulateStepsActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = SimulateStepsResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
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
