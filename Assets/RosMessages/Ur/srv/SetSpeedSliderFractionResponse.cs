//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur
{
    [Serializable]
    public class SetSpeedSliderFractionResponse : Message
    {
        public const string k_RosMessageName = "ur_msgs/SetSpeedSliderFraction";
        public override string RosMessageName => k_RosMessageName;

        public bool success;

        public SetSpeedSliderFractionResponse()
        {
            this.success = false;
        }

        public SetSpeedSliderFractionResponse(bool success)
        {
            this.success = success;
        }

        public static SetSpeedSliderFractionResponse Deserialize(MessageDeserializer deserializer) => new SetSpeedSliderFractionResponse(deserializer);

        private SetSpeedSliderFractionResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "SetSpeedSliderFractionResponse: " +
            "\nsuccess: " + success.ToString();
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
