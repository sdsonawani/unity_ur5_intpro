using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;



public class CamPoseSubscriber: MonoBehaviour{

    public GameObject Cylinder;
    public GameObject Cube;
    public Camera POV = new Camera();
    ROSConnection ros;
    public string topicName = "pov_pose";
    public string camTopicName = "pov_image";
    public float trans_x,trans_y,trans_z;
    public float quat_w,quat_x,quat_y,quat_z;
    public float offset  = 0;
    public HeaderMsg msg_ = new HeaderMsg();
    Camera cam;

    void Start(){
        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CamPoseMsg>(topicName, PoseCallback);
        // ros.RegisterPublisher<ImageMsg>(camTopicName);
        // POV.orthographic = true;

        Vector3 c_trans = new Vector3(0.027f,0.65f,0.384f);
        Quaternion c_quat = Quaternion.Euler(0,0,0);
        Cylinder.transform.SetPositionAndRotation(c_trans,c_quat);
        Vector3 cube_trans = new Vector3(0.027f ,0.65f +0.15f,0.384f);
        Quaternion cube_quat = Quaternion.Euler(0,0,0);
        Cube.transform.SetPositionAndRotation(cube_trans,cube_quat);
        // POV.transform.SetParent(Cube.transform);
        // cam  = Instantiate(POV);
        POV.enabled = true;
        // POV.tag = "POV_camera"


    }

    void PoseCallback(CamPoseMsg msg){
        trans_x = (float)msg.x;
        trans_y = (float)msg.y;
        trans_z = (float)msg.z;
        quat_x  = (float)msg.x_;
        quat_y  = (float)msg.y_;
        quat_z  = (float)msg.z_;
        quat_w  = (float)msg.w_;
    }

    public void BgrToRgb(byte[] data) {
        for (int i = 0; i < data.Length; i += 3)
        {
            byte dummy = data[i];
            data[i] = data[i + 2];
            data[i + 2] = dummy;
        }
    }

    void Update(){
        
        // Camera cam = Instantiate(POV);
        // var currentRT = RenderTexture.active;
        // RenderTexture.active = cam.targetTexture;
        // cam.Render();
        // Texture2D image =  new Texture2D(cam.targetTexture.width,cam.targetTexture.height);
        // image.ReadPixels(new Rect(0,0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        // image.Apply();
        // RenderTexture.active = currentRT;
        // msg_.frame_id = "pov_frame";
        // ImageMsg imagemsg = image.ToImageMsg(msg_);
        // ros.Publish(camTopicName, imagemsg);



        // POV.transform.Translate(0,offset,0);
        // Vector3 trans = new Vector3(0,1,1);
        // Vector3 trans = new Vector3(trans_x+0.1f,trans_y+0.1f,trans_z+0.1f);

        Vector3 trans = new Vector3(trans_x,trans_y + 1.0f,trans_z);
        Vector3 trans1 = new Vector3(trans_x,trans_y,trans_z);
        Vector3 cube_trans = new Vector3(0.027f ,0.65f +0.15f,0.384f);
        Vector3 relative_pose = cube_trans - trans;


        
        var rad   = relative_pose.magnitude;
        var theta = Math.Atan(relative_pose[1]/relative_pose[0]);
        var phi   = Math.Acos(relative_pose[2]/rad);
        Debug.Log(rad);

        var rad2  = 1.5f;
        var x_    = rad2 * Math.Cos(theta) * Math.Sin(phi);
        var y_    = rad2 * Math.Sin(theta) * Math.Sin(phi);
        var z_    = rad2 * Math.Cos(phi);
        var new_xyz = new Vector3((float)x_,(float)y_, (float)z_);
        var new_pose = new_xyz - cube_trans;


        // Vector3 relative_pose = trans - cube_trans;
        Quaternion rotation = Quaternion.LookRotation(relative_pose);
        // Vector3 trans = new Vector3(trans_x,trans_y,trans_z);
        // Quaternion quat  = new Quaternion(0,0,0,1);
        Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
        // Quaternion quat  = Quaternion.Euler(135,0,180);
        // POV.transform.SetLocalPositionAndRotation(trans,quat);
        // POV.transform.localPosition = new Vector3(0.0f,1.2f,1.0f);
        // POV.transform.localRotation = quat;
        // POV.transform.SetLocalPositionAndRotation(trans,quat);

        POV.transform.SetPositionAndRotation(trans,quat);
        // POV.transform.SetPositionAndRotation(trans,rotation);
        // POV.transform.SetPositionAndRotation(new_pose,quat);

        Vector3 lookat_axis = new Vector3(0,1,0);
        // Vector3 lookat_axis = new Vector3(0.5f,0.5f,0.5f);
        POV.transform.LookAt(Cube.transform,lookat_axis);
        // POV.transform.LookAt(Cube.transform, Vector3.up);
        // POV.transform.LookAt(Cylinder.transform);
        // POV.transform.position(0,1,0);
        // offset = offset - 0.001f;
        // Debug.Log(string.Format("translation: [{0},{1},{2}]",trans_x,trans_y,trans_z));    
        // Debug.Log(string.Format("Camera properties: [{0}]",POV.aspect));    
        // Debug.Log(string.Format("Camera properties: [{0}]",POV.tag));    
        // Debug.Log(string.Format("Camera width: [{0}]",POV.targetTexture.width));    
    }
}