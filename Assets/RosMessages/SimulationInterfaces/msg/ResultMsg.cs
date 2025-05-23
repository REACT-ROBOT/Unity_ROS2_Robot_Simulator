//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.SimulationInterfaces
{
    [Serializable]
    public class ResultMsg : Message
    {
        public const string k_RosMessageName = "simulation_interfaces/Result";
        public override string RosMessageName => k_RosMessageName;

        //  Result code and message for service calls.
        //  Note that additional results for specific services can defined within them using values above 100.
        public const byte RESULT_FEATURE_UNSUPPORTED = 0; //  Feature is not supported by the simulator, check GetSimulatorFeatures.
        //  While feature support can sometimes be deduced from presence of corresponding
        //  service / action interface, in other cases it is about supporting certain
        //  call parameters, formats and options within interface call.
        public const byte RESULT_OK = 1;
        public const byte RESULT_NOT_FOUND = 2; //  No match for input (such as when entity name does not exist).
        public const byte RESULT_INCORRECT_STATE = 3; //  Simulator is in an incorrect state for this interface call, e.g. a service
        //  requires paused state but the simulator is not paused.
        public const byte RESULT_OPERATION_FAILED = 4; //  Request could not be completed successfully even though feature is supported
        //  and the input is correct; check error_message for details.
        //  Implementation rule: check extended codes for called service
        //   (e.g. SpawnEntity) to provide a return code which is more specific.
        public byte result;
        //  Result to be checked on return from service interface call
        public string error_message;
        //  Additional error description when useful.

        public ResultMsg()
        {
            this.result = 0;
            this.error_message = "";
        }

        public ResultMsg(byte result, string error_message)
        {
            this.result = result;
            this.error_message = error_message;
        }

        public static ResultMsg Deserialize(MessageDeserializer deserializer) => new ResultMsg(deserializer);

        private ResultMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.result);
            deserializer.Read(out this.error_message);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.result);
            serializer.Write(this.error_message);
        }

        public override string ToString()
        {
            return "ResultMsg: " +
            "\nresult: " + result.ToString() +
            "\nerror_message: " + error_message.ToString();
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
