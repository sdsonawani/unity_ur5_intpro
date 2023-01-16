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

    public GameObject Table;
    // public GameObject Cube_1;
    public GameObject Cube_2;
    public GameObject Cube_2_base;
    public GameObject Cube_2_basec1;
    public GameObject Cube_2_basec2;
    public GameObject Cube_2_basec3;
    // public GameObject Cube_3;
    public Camera POV = new Camera();
    public Camera New_POV = new Camera();
    ROSConnection ros;
    public string topicName = "pov_pose";
    public string camTopicName = "pov_image";
    public float trans_x,trans_y,trans_z;
    public float quat_w,quat_x,quat_y,quat_z;
    public float offset  = 0;
    public HeaderMsg msg_ = new HeaderMsg();
    Camera cam;



    void Start(){

        // Cube_1 = GameObject.Find("Cube_1");
        Cube_2      = GameObject.Find("Cube_2");
        Cube_2_base = GameObject.Find("Base_Plate");
        Cube_2_basec1 = GameObject.Find("Base_Plate_c1");
        Cube_2_basec2 = GameObject.Find("Base_Plate_c2");
        Cube_2_basec3 = GameObject.Find("Base_Plate_c3");
        Material ProjSurfMaterial = Cube_2_base.GetComponent<Renderer>().material;
        ProjSurfMaterial.color = Color.red;

        Material ProjSurfMateria_c1 = Cube_2_basec1.GetComponent<Renderer>().material;
        Color customColor = new Color(0.4f, 0.9f, 0.7f, 1.0f);
        ProjSurfMateria_c1.color = customColor;

        Material ProjSurfMaterial_c2 = Cube_2_basec2.GetComponent<Renderer>().material;
        ProjSurfMaterial_c2.color = Color.green;

        Material ProjSurfMaterial_c3 = Cube_2_basec3.GetComponent<Renderer>().material;
        ProjSurfMaterial_c3.color = Color.blue;

        // Shader shader_  = 
        // ProjSurfMaterial.SetTexture("_MainTex",testText);
        // ProjSurfMaterial.SetTexture("_MainTex",ShadowImage);
        // Cube_3 = GameObject.Find("Cube_3");
        Table  = GameObject.Find("table_top");
        POV    = GameObject.Find("POV").GetComponent<Camera>();
        New_POV = GameObject.Find("New_POV").GetComponent<Camera>();

        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CamPoseMsg>(topicName, PoseCallback);
        ros.RegisterPublisher<CamPoseMsg>("pov_rel_pose");

        // Vector3 cube1_trans = new Vector3(0.2f , 0.40f, 0.8f);
        // Quaternion cube1_quat = Quaternion.Euler(0, 0, 0);
        // Cube_1.transform.SetPositionAndRotation(cube1_trans,cube1_quat);
        // Collider Cube_1_Collider = Cube_1.GetComponent<Collider>();
        // Cube_1_Collider.enabled = !Cube_1_Collider.enabled;

        Vector3 cube2_trans = new Vector3(0.0f , 0.40f,0.6f);
        Quaternion cube2_quat = Quaternion.Euler(0, 0, 0);
        Cube_2.transform.SetPositionAndRotation(cube2_trans,cube2_quat);


        Vector3 cube2_base_trans = new Vector3(0.0f , 0.35f,0.6f);
        Quaternion cube2_base_quat = Quaternion.Euler(0, 0, 0);
        Cube_2_base.transform.SetPositionAndRotation(cube2_base_trans,cube2_base_quat);


        // Vector3 cube2_base_c1_trans = new Vector3(-0.5f , 0.35f, -0.3f);
        // Vector3 cube2_base_c2_trans = new Vector3(0.5f , 0.35f, -0.3f);
        // Vector3 cube2_base_c3_trans = new Vector3( 0.5f , 0.35f,  0.3f);    

        // Cube_2_basec1.transform.SetPositionAndRotation(cube2_base_c1_trans,cube2_base_quat);
        // Cube_2_basec2.transform.SetPositionAndRotation(cube2_base_c2_trans,cube2_base_quat);
        // Cube_2_basec3.transform.SetPositionAndRotation(cube2_base_c3_trans,cube2_base_quat);


        Collider Cube_2_Collider = Cube_2.GetComponent<Collider>();
        Collider Cube_2_base_Collider = Cube_2_base.GetComponent<Collider>();
        Cube_2_Collider.enabled = !Cube_2_Collider.enabled;
        Cube_2_base_Collider.enabled = !Cube_2_base_Collider.enabled;

        // Vector3 cube3_trans = new Vector3(-0.2f, 0.4f, 0.8f);
        // Quaternion cube3_quat = Quaternion.Euler(0, 0, 0);
        // Cube_3.transform.SetPositionAndRotation(cube3_trans,cube3_quat);
        // Collider Cube_3_Collider = Cube_3.GetComponent<Collider>();
        // Cube_3_Collider.enabled = !Cube_3_Collider.enabled;

        // Trying other way around 

        // Vector3 POV_Trans = new Vector3(0.0f , 0.0f,0.0f);
        // Quaternion POV_Quat = Quaternion.Euler(0, 0, 180);
        // POV.transform.SetPositionAndRotation(POV_Trans,POV_Quat);
        // Collider Cube_2_Collider = Cube_2.GetComponent<Collider>();

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

        Vector3 trans    = new Vector3(trans_x,trans_y + 0.40f,trans_z + 0.6f);
        // Vector3 trans    = new Vector3(trans_x,trans_y,trans_z);
        Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
        POV.transform.SetPositionAndRotation(trans,quat);
        New_POV.transform.SetPositionAndRotation(trans,quat);
        Vector3 lookat_axis = new Vector3(0,1,0);
        POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        New_POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        
        Quaternion quat_base = Cube_2_base.transform.rotation;
        Quaternion quat_pov  = POV.transform.rotation;


        Quaternion rel_quat  = quat_pov * Quaternion.Inverse(quat_base);
        Vector3 rel_trans    = POV.transform.position - Cube_2_base.transform.position;

        CamPoseMsg pose  = new CamPoseMsg();
        pose.x = rel_trans[0];
        pose.y = rel_trans[1];
        pose.z = rel_trans[2];

        pose.x_ = rel_quat.x;
        pose.y_ = rel_quat.y;
        pose.z_ = rel_quat.z;
        pose.w_ = rel_quat.w;
        ros.Publish("pov_rel_pose", pose);


        // Vector3 euler = quat.eulerAngles;
        // Debug.Log(string.Format("x: {0}, y:{1}, z:{2}",euler[0],euler[1],euler[2]));
        // Quaternion quat_1 = Quaternion.Euler(euler[0],euler[1],euler[2]);
        // Quaternion quat_1 = Quaternion.Euler(euler[2],euler[1],euler[0]);
        // Quaternion quat_1 = Quaternion.Euler(0, - euler[1], 0);
        // Quaternion quat_1 = Quaternion.Euler(0,-euler[1],-euler[2]);

        // POV.transform.position = trans;
        // Cube_2.transform.rotation = quat_1;
        // Cube_2.transform.rotation = quat;
        // Cube_2.transform.position = trans;
        // Cube_2.transform.SetPositionAndRotation(trans,quat);

        /* Enable look at to fix the camera look to specified
        object */
        // Vector3 lookat_axis = new Vector3(0,1,0);
        // Vector3 lookat_axis = new Vector3(0,1,0);
        // POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        // POV.transform.LookAt(Cube_2.transform,lookat_axis); 


        // Test Code
        // ============================================================
        // Vector3 cube_trans = new Vector3(0.0f ,0.8f,0.384f);
        // Vector3 relative_pose = (cube_trans - trans) * 0.5f;        
        // var rad   = relative_pose.magnitude;
        // var theta = Math.Atan(relative_pose[1]/relative_pose[0]);
        // var phi   = Math.Acos(relative_pose[2]/rad);
        // var rad2  = 1.5f;
        // var x_    = rad2 * Math.Cos(theta) * Math.Sin(phi);
        // var y_    = rad2 * Math.Sin(theta) * Math.Sin(phi);
        // var z_    = rad2 * Math.Cos(phi);
        // var new_xyz = new Vector3((float)x_,(float)y_, (float)z_);
        // var new_pose = new_xyz - cube_trans;
        // Quaternion quat  = Quaternion.Euler(135,0,180);
        // POV.transform.SetLocalPositionAndRotation(trans,quat);
        // POV.transform.localPosition = new Vector3(0.0f,1.2f,1.0f);
        // POV.transform.localRotation = quat;
        // POV.transform.SetLocalPositionAndRotation(trans,quat);

        // POV.transform.SetPositionAndRotation(trans,rotation);
        // POV.transform.SetPositionAndRotation(new_pose,quat);
        // Vector3 lookat_axis = new Vector3(0,1,0);
        // POV.transform.LookAt(Cube.transform,lookat_axis); 



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