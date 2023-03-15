using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter.Control;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;



public class Teleop: MonoBehaviour{

    public GameObject Cube ;
    ROSConnection ros;
    public string teleopTopicName = "obj_pose";
    public float trans_x,trans_y,trans_z;
    public float quat_w,quat_x,quat_y,quat_z;
    public float offset  = 0;
    public HeaderMsg msg_ = new HeaderMsg();

    private float x_min = -0.67f;
    private float x_max =  0.59f;
    private float z_min =  0.05f;
    private float z_max =  0.80f;
    private float y_min =  0.29f;
    private float y_max =  0.28f;
  
    
    void PoseCallback(TeleopPoseMsg msg){
        trans_x = (float)msg.x;
        trans_y = (float)msg.y + 0.045f;
        trans_z = (float)msg.z;
        quat_x  = (float)msg.x_;
        quat_y  = (float)msg.y_;
        quat_z  = (float)msg.z_;
        quat_w  = (float)msg.w_;
    }

    void Start(){
        Cube       = GameObject.Find("Cube_3");
        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TeleopPoseMsg>(teleopTopicName, PoseCallback);
        Collider Cube_Collider = Cube.GetComponent<Collider>();
        Cube_Collider.enabled = !Cube_Collider.enabled;
    }


    // void PoseCallback()

    void Update(){

        if (trans_y < 0.35f){
            trans_y = 0.35f;
        }
        // trans_y = 0.35f;
        Vector3 trans = new Vector3(trans_x,trans_y,trans_z);
        Quaternion quat  = new Quaternion(0.0f,0.0f,0.0f,1.0f);
        // Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
        Cube.transform.SetPositionAndRotation(trans,quat);

    }
}