//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboteqDiff
{
    [Serializable]
    public class RequestChargeActivateRequest : Message
    {
        public const string k_RosMessageName = "roboteq_diff_msgs/RequestChargeActivate";
        public override string RosMessageName => k_RosMessageName;

        public bool activate_charging;

        public RequestChargeActivateRequest()
        {
            this.activate_charging = false;
        }

        public RequestChargeActivateRequest(bool activate_charging)
        {
            this.activate_charging = activate_charging;
        }

        public static RequestChargeActivateRequest Deserialize(MessageDeserializer deserializer) => new RequestChargeActivateRequest(deserializer);

        private RequestChargeActivateRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.activate_charging);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.activate_charging);
        }

        public override string ToString()
        {
            return "RequestChargeActivateRequest: " +
            "\nactivate_charging: " + activate_charging.ToString();
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