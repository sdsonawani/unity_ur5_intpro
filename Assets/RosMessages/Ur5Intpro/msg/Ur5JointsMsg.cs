//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur5Intpro
{
    [Serializable]
    public class Ur5JointsMsg : Message
    {
        public const string k_RosMessageName = "ur5_intpro/Ur5Joints";
        public override string RosMessageName => k_RosMessageName;

        public double[] ur_joints_unity;

        public Ur5JointsMsg()
        {
            this.ur_joints_unity = new double[6];
        }

        public Ur5JointsMsg(double[] ur_joints_unity)
        {
            this.ur_joints_unity = ur_joints_unity;
        }

        public static Ur5JointsMsg Deserialize(MessageDeserializer deserializer) => new Ur5JointsMsg(deserializer);

        private Ur5JointsMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.ur_joints_unity, sizeof(double), 6);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.ur_joints_unity);
        }

        public override string ToString()
        {
            return "Ur5JointsMsg: " +
            "\nur_joints_unity: " + System.String.Join(", ", ur_joints_unity.ToList());
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