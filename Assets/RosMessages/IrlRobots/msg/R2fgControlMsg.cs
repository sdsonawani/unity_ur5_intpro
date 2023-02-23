//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.IrlRobots
{
    [Serializable]
    public class R2fgControlMsg : Message
    {
        public const string k_RosMessageName = "irl_robots/r2fgControl";
        public override string RosMessageName => k_RosMessageName;

        public bool activate;
        public bool go;
        public bool autorelease_open;
        public bool autorelease;
        public byte position;
        public byte speed;
        public byte force;

        public R2fgControlMsg()
        {
            this.activate = false;
            this.go = false;
            this.autorelease_open = false;
            this.autorelease = false;
            this.position = 0;
            this.speed = 0;
            this.force = 0;
        }

        public R2fgControlMsg(bool activate, bool go, bool autorelease_open, bool autorelease, byte position, byte speed, byte force)
        {
            this.activate = activate;
            this.go = go;
            this.autorelease_open = autorelease_open;
            this.autorelease = autorelease;
            this.position = position;
            this.speed = speed;
            this.force = force;
        }

        public static R2fgControlMsg Deserialize(MessageDeserializer deserializer) => new R2fgControlMsg(deserializer);

        private R2fgControlMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.activate);
            deserializer.Read(out this.go);
            deserializer.Read(out this.autorelease_open);
            deserializer.Read(out this.autorelease);
            deserializer.Read(out this.position);
            deserializer.Read(out this.speed);
            deserializer.Read(out this.force);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.activate);
            serializer.Write(this.go);
            serializer.Write(this.autorelease_open);
            serializer.Write(this.autorelease);
            serializer.Write(this.position);
            serializer.Write(this.speed);
            serializer.Write(this.force);
        }

        public override string ToString()
        {
            return "R2fgControlMsg: " +
            "\nactivate: " + activate.ToString() +
            "\ngo: " + go.ToString() +
            "\nautorelease_open: " + autorelease_open.ToString() +
            "\nautorelease: " + autorelease.ToString() +
            "\nposition: " + position.ToString() +
            "\nspeed: " + speed.ToString() +
            "\nforce: " + force.ToString();
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
