//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur5Intpro
{
    [Serializable]
    public class RGBXYSEGMsg : Message
    {
        public const string k_RosMessageName = "ur5_intpro/RGBXYSEG";
        public override string RosMessageName => k_RosMessageName;

        public Sensor.ImageMsg Image;
        public float[] x_image;
        public float[] y_image;
        public float[] x_world;
        public float[] y_world;
        public float[] z_world;
        public float[] joint_angles;
        public string[] obj_names;
        public long data_counter;

        public RGBXYSEGMsg()
        {
            this.Image = new Sensor.ImageMsg();
            this.x_image = new float[0];
            this.y_image = new float[0];
            this.x_world = new float[0];
            this.y_world = new float[0];
            this.z_world = new float[0];
            this.joint_angles = new float[7];
            this.obj_names = new string[0];
            this.data_counter = 0;
        }

        public RGBXYSEGMsg(Sensor.ImageMsg Image, float[] x_image, float[] y_image, float[] x_world, float[] y_world, float[] z_world, float[] joint_angles, string[] obj_names, long data_counter)
        {
            this.Image = Image;
            this.x_image = x_image;
            this.y_image = y_image;
            this.x_world = x_world;
            this.y_world = y_world;
            this.z_world = z_world;
            this.joint_angles = joint_angles;
            this.obj_names = obj_names;
            this.data_counter = data_counter;
        }

        public static RGBXYSEGMsg Deserialize(MessageDeserializer deserializer) => new RGBXYSEGMsg(deserializer);

        private RGBXYSEGMsg(MessageDeserializer deserializer)
        {
            this.Image = Sensor.ImageMsg.Deserialize(deserializer);
            deserializer.Read(out this.x_image, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.y_image, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.x_world, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.y_world, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.z_world, sizeof(float), deserializer.ReadLength());
            deserializer.Read(out this.joint_angles, sizeof(float), 7);
            deserializer.Read(out this.obj_names, deserializer.ReadLength());
            deserializer.Read(out this.data_counter);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.Image);
            serializer.WriteLength(this.x_image);
            serializer.Write(this.x_image);
            serializer.WriteLength(this.y_image);
            serializer.Write(this.y_image);
            serializer.WriteLength(this.x_world);
            serializer.Write(this.x_world);
            serializer.WriteLength(this.y_world);
            serializer.Write(this.y_world);
            serializer.WriteLength(this.z_world);
            serializer.Write(this.z_world);
            serializer.Write(this.joint_angles);
            serializer.WriteLength(this.obj_names);
            serializer.Write(this.obj_names);
            serializer.Write(this.data_counter);
        }

        public override string ToString()
        {
            return "RGBXYSEGMsg: " +
            "\nImage: " + Image.ToString() +
            "\nx_image: " + System.String.Join(", ", x_image.ToList()) +
            "\ny_image: " + System.String.Join(", ", y_image.ToList()) +
            "\nx_world: " + System.String.Join(", ", x_world.ToList()) +
            "\ny_world: " + System.String.Join(", ", y_world.ToList()) +
            "\nz_world: " + System.String.Join(", ", z_world.ToList()) +
            "\njoint_angles: " + System.String.Join(", ", joint_angles.ToList()) +
            "\nobj_names: " + System.String.Join(", ", obj_names.ToList()) +
            "\ndata_counter: " + data_counter.ToString();
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
