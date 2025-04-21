using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.SimulationInterfaces
{
    public class SimulateStepsActionGoal : ActionGoal<SimulateStepsGoal>
    {
        public const string k_RosMessageName = "simulation_interfaces/SimulateStepsActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public SimulateStepsActionGoal() : base()
        {
            this.goal = new SimulateStepsGoal();
        }

        public SimulateStepsActionGoal(HeaderMsg header, GoalIDMsg goal_id, SimulateStepsGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static SimulateStepsActionGoal Deserialize(MessageDeserializer deserializer) => new SimulateStepsActionGoal(deserializer);

        SimulateStepsActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = SimulateStepsGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
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
