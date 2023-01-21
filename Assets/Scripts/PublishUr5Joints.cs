using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using System;

public class PublishUr5Joints : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "unity_ur5/joints";
    public string topicNameShadow = "unity_ur5_shadow/joints";
    private const int num_joints = 6;


    // The game object
    public GameObject ur5;
    public GameObject ur5_1;

    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.01f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    public static readonly string[] LinkNames =
    {
        "world/dummy_link/base_link/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link",
        "/wrist_2_link", "/wrist_3_link"
    };

    private UrdfJointRevolute[] m_jab;
    private UrdfJointRevolute[] m_Shadow_jab;

    private void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Ur5JointsMsg>(topicName);
        ros.RegisterPublisher<Ur5JointsMsg>(topicNameShadow);
        // Debug.developerConsoleVisible = true;

        ur5 = GameObject.Find("ur5");
        ur5_1 = GameObject.Find("ur5_1");


        m_jab = new UrdfJointRevolute[num_joints];
        m_Shadow_jab = new UrdfJointRevolute[num_joints];
        var link_names = string.Empty;
        for (var i = 0; i < num_joints; i++)
        {
            link_names += LinkNames[i];
            m_jab[i] = ur5.transform.Find(link_names).GetComponent<UrdfJointRevolute>();
            m_Shadow_jab[i] = ur5_1.transform.Find(link_names).GetComponent<UrdfJointRevolute>();
            // Debug.Log(string.Format("{0}",link_names));
        }
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            // double[] j_ = new double[] {0.1,0.1,0.1,0.1,0.1,0.1};
            var j_ = new double[6];
            var j_Shadow = new double[6];
            for (var i = 0; i < num_joints; i++)
                if (i == 1 || i == 3)
                {
                    j_[i] = m_jab[i].GetPosition() - Math.PI / 2.0;
                    j_Shadow[i] = m_Shadow_jab[i].GetPosition() - Math.PI / 2.0;
                }
                else
                {
                    j_[i] = m_jab[i].GetPosition();
                    j_Shadow[i] = m_Shadow_jab[i].GetPosition();
                }

            // Debug.Log(string.Format("{0}",j_[i]));
            var joints = new Ur5JointsMsg(j_);
            var jointsShadow = new Ur5JointsMsg(j_Shadow);


            ros.Publish(topicName, joints);
            ros.Publish(topicNameShadow, jointsShadow);
            timeElapsed = 0;
        }
    }
}