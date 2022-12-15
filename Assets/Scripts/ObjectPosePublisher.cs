using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.ROSTCPConnector;


public class ObjectPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_obj/pose";

    // The game object
    public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.01f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CamPoseMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            cube.transform.rotation = Random.rotation;
            CamPoseMsg cubePos = new CamPoseMsg(cube.transform.position.x,
                                                cube.transform.position.y,
                                                cube.transform.position.z,
                                                cube.transform.rotation.x,
                                                cube.transform.rotation.y,
                                                cube.transform.rotation.z,
                                                cube.transform.rotation.w);
            // cubePos.pose.position.x = cube.transform.position.x;
            // cubePos.pose.position.y = cube.transform.position.y;
            // cubePos.pose.position.z = cube.transform.position.z;
            // cubePos.pose.orientation.x = 0;
            // cubePos.pose.orientation.y = 0;
            // cubePos.pose.orientation.z = 0;
            // cubePos.pose.orientation.w = 1;

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, cubePos);
            timeElapsed = 0;
        }
    }
}