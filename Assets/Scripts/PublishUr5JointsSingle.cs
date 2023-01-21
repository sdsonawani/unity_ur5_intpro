using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using System;

public class PublishUr5JointsSingle : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "unity_ur5/joints";
    private const int num_joints = 6;


    // The game object
    public GameObject ur5;

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
        // Debug.developerConsoleVisible = true;

        m_jab = new UrdfJointRevolute[num_joints];
        var link_names = string.Empty;
        for (var i = 0; i < num_joints; i++)
        {
            link_names += LinkNames[i];
            m_jab[i] = ur5.transform.Find(link_names).GetComponent<UrdfJointRevolute>();
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
            for (var i = 0; i < num_joints; i++)
                if (i == 1 || i == 3)
                    j_[i] = m_jab[i].GetPosition() - Math.PI / 2.0;
                else
                    j_[i] = m_jab[i].GetPosition();
            // Debug.Log(string.Format("{0}",j_[i]));
            var joints = new Ur5JointsMsg(j_);


            ros.Publish(topicName, joints);
            timeElapsed = 0;
        }
    }
}