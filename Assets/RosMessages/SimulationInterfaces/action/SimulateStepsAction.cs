using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.SimulationInterfaces
{
    public class SimulateStepsAction : Action<SimulateStepsActionGoal, SimulateStepsActionResult, SimulateStepsActionFeedback, SimulateStepsGoal, SimulateStepsResult, SimulateStepsFeedback>
    {
        public const string k_RosMessageName = "simulation_interfaces/SimulateStepsAction";
        public override string RosMessageName => k_RosMessageName;


        public SimulateStepsAction() : base()
        {
            this.action_goal = new SimulateStepsActionGoal();
            this.action_result = new SimulateStepsActionResult();
            this.action_feedback = new SimulateStepsActionFeedback();
        }

        public static SimulateStepsAction Deserialize(MessageDeserializer deserializer) => new SimulateStepsAction(deserializer);

        SimulateStepsAction(MessageDeserializer deserializer)
        {
            this.action_goal = SimulateStepsActionGoal.Deserialize(deserializer);
            this.action_result = SimulateStepsActionResult.Deserialize(deserializer);
            this.action_feedback = SimulateStepsActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
