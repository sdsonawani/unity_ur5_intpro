//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboteqDiff
{
    [Serializable]
    public class EMOResponse : Message
    {
        public const string k_RosMessageName = "roboteq_diff_msgs/EMO";
        public override string RosMessageName => k_RosMessageName;

        public bool complete;

        public EMOResponse()
        {
            this.complete = false;
        }

        public EMOResponse(bool complete)
        {
            this.complete = complete;
        }

        public static EMOResponse Deserialize(MessageDeserializer deserializer) => new EMOResponse(deserializer);

        private EMOResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.complete);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.complete);
        }

        public override string ToString()
        {
            return "EMOResponse: " +
            "\ncomplete: " + complete.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
