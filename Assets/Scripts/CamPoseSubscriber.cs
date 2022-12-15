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
        // POV.transform.SetParent(Cylinder.transform);
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
        Vector3 trans = new Vector3(trans_x,trans_y + 0.5f,trans_z);
        // Quaternion quat  = new Quaternion(0,0,0,1);
        Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
        // Quaternion quat  = Quaternion.Euler(135,0,180);
        // POV.transform.SetLocalPositionAndRotation(trans,quat);
        // POV.transform.localPosition = new Vector3(0.0f,1.2f,1.0f);
        // POV.transform.localRotation = quat;
        // POV.transform.SetLocalPositionAndRotation(trans,quat);

        POV.transform.SetPositionAndRotation(trans,quat);
        // POV.transform.position(0,1,0);
        // offset = offset - 0.001f;
        // Debug.Log(string.Format("translation: [{0},{1},{2}]",trans_x,trans_y,trans_z));    
        // Debug.Log(string.Format("Camera properties: [{0}]",POV.aspect));    
        // Debug.Log(string.Format("Camera properties: [{0}]",POV.tag));    
        // Debug.Log(string.Format("Camera width: [{0}]",POV.targetTexture.width));    
    }
}