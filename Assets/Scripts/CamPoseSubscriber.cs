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
using System.Collections;
using System.Collections.Generic;


public class CamPoseSubscriber: MonoBehaviour{

    public bool use_headtrack = true;
    public GameObject Cube_1;
    public GameObject Cube_2;
    public GameObject Cube_2_base;
    public GameObject Cube_2_base1;
    public GameObject Cube_2_basec1;
    public GameObject Cube_2_basec2;
    public GameObject Cube_2_basec3;
    public GameObject Cube_2_basec4;

    public Camera POV = new Camera();
    public Camera New_POV = new Camera();
    // public Camera New_POV_1 = new Camera();

    public float table_x = 0.0f;
    public float table_y = 0.30f;
    public float table_z = 0.47f;

    // temp 
    public float cube_height = 0.05f;

    ROSConnection ros;
    public string topicName = "pov_pose";
    public string htopicName = "human_pose";
    public string imageAndSrcmatTopic = "image_and_src_mat";
    public float trans_x,trans_y,trans_z;
    public float quat_w,quat_x,quat_y,quat_z;

    public float htrans_x,htrans_y,htrans_z;
    public float hquat_w,hquat_x,hquat_y,hquat_z;

    public float offset  = 0;
    public HeaderMsg msg_ = new HeaderMsg();
    Camera cam;
    public Shader shade;
    public RenderTexture renderTexture;
    public int Pwdith  = 1920;
    public int Pheight = 1080;

        
    private List<GameObject> objects;

    void ApplyMaterial(GameObject obj, Color color){
        Material mat = obj.GetComponent<Renderer>().material;
        mat.shader = shade;
        mat.color = color;

    }

    void ApplyPRAndCollision(GameObject obj, Vector3 trans, Quaternion quat, bool Collide = false){
        obj.transform.SetPositionAndRotation(trans,quat);
        Collider obj_collider = obj.GetComponent<Collider>();
        if (Collide == false){
            obj_collider.enabled = !obj_collider.enabled;
        }
        else{
            obj_collider.enabled = true;
        }

    }

    void Start(){

        Cube_1        = GameObject.Find("Cube_1");
        Cube_2        = GameObject.Find("Cube_2");
        Cube_2_base   = GameObject.Find("Base_Plate");
        Cube_2_base1   = GameObject.Find("Checkerboard");
        Cube_2_basec1 = GameObject.Find("Base_Plate_c1");
        Cube_2_basec2 = GameObject.Find("Base_Plate_c2");
        Cube_2_basec3 = GameObject.Find("Base_Plate_c3");
        Cube_2_basec4 = GameObject.Find("Base_Plate_c4");
        objects =  new List<GameObject> (){Cube_2_basec1, Cube_2_basec4, Cube_2_basec3, Cube_2_basec2 };
        shade = Shader.Find("Unlit/Color");
        Color customColor = new Color(0.4f, 0.9f, 0.7f, 1.0f);
        Color new_color   = new Color(1f, 1f, 1f, 1.0f);
        ApplyMaterial(Cube_2_base   , new_color);
        ApplyMaterial(Cube_2_basec1 , Color.red);
        ApplyMaterial(Cube_2_basec2,  Color.green);
        ApplyMaterial(Cube_2_basec3,  Color.blue);
        ApplyMaterial(Cube_2_basec4,  Color.cyan);
        
    
        POV       = GameObject.Find("POV").GetComponent<Camera>();
        // New_POV   = GameObject.Find("POV_1").GetComponent<Camera>();
        // New_POV_1 = GameObject.Find("New_POV_1").GetComponent<Camera>();
        Pwdith  = POV.pixelWidth;
        Pheight = POV.pixelHeight;
        POV.clearFlags = CameraClearFlags.SolidColor;


        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CamPoseMsg>(topicName, PoseCallback);
        ros.Subscribe<CamPoseMsg>(htopicName, HPoseCallback);
        ros.RegisterPublisher<CamPoseMsg>("pov_rel_pose");
        ros.RegisterPublisher<ImageSrcMatMsg>(imageAndSrcmatTopic);


        // Base Plane
        Vector3 cube2_base_trans = new Vector3(table_x , table_y, table_z);
        Vector3 cube2_base_trans1 = new Vector3(table_x , table_y+10.01f, table_z);
        Quaternion cube2_base_quat = Quaternion.Euler(0, 0, 0);
        ApplyPRAndCollision(Cube_2_base, cube2_base_trans, cube2_base_quat);
        ApplyPRAndCollision(Cube_2_base1, cube2_base_trans1, cube2_base_quat);
        
        // RGB Planes
        // Vector3 cube2_base_c1_trans = new Vector3(-0.5f , table_y, 0.2f);
        // Vector3 cube2_base_c2_trans = new Vector3( 0.5f , table_y, 0.2f);
        // Vector3 cube2_base_c3_trans = new Vector3( 0.5f , table_y, 1.0f);    
        // Vector3 cube2_base_c4_trans = new Vector3(- 0.5f , table_y, 1.0f);    

        Vector3 cube2_base_c1_trans = new Vector3(- 0.75f, table_y, 0.075f);
        Vector3 cube2_base_c2_trans = new Vector3(  0.75f, table_y, 0.075f);
        Vector3 cube2_base_c3_trans = new Vector3(  0.75f, table_y, 0.875f);    
        Vector3 cube2_base_c4_trans = new Vector3(- 0.75f, table_y, 0.875f);
        ApplyPRAndCollision(Cube_2_basec1, cube2_base_c1_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec2, cube2_base_c2_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec3, cube2_base_c3_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec4, cube2_base_c4_trans, Quaternion.Euler(0,0,0));


        Vector3 cube1_trans = new Vector3( (table_x + 0.2f) , (table_y + cube_height), table_z);
        Quaternion cube1_quat = Quaternion.Euler(0, 0, 90f);
        ApplyPRAndCollision(Cube_1, cube1_trans, cube1_quat);

        Vector3 cube2_trans = new Vector3(-(table_x + 0.2f),  (table_y + cube_height), table_z);
        Quaternion cube2_quat = Quaternion.Euler(0, 0, 0);
        ApplyPRAndCollision(Cube_2, cube2_trans, cube2_quat);

        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UNorm);
        //  renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        // renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        // renderTexture      = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UInt);
        renderTexture.Create();
    
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

    void HPoseCallback(CamPoseMsg msg){
        htrans_x = (float)msg.x;
        htrans_y = (float)msg.y;
        htrans_z = (float)msg.z;
        hquat_x  = (float)msg.x_;
        hquat_y  = (float)msg.y_;
        hquat_z  = (float)msg.z_;
        hquat_w  = (float)msg.w_;
    }

    public void BgrToRgb(byte[] data) {
        for (int i = 0; i < data.Length; i += 3)
        {
            byte dummy = data[i];
            data[i] = data[i + 2];
            data[i + 2] = dummy;
        }
    }


    public void publish_imageNsrcmat(){
        // POV.cullingMask = LayerMask.GetMask("render_layer_1");
        
        double[] xs = new double[4];
        double[] ys = new double[4];
        for (int i = 0; i < 4; i++){
            Vector3 screenPos = POV.WorldToScreenPoint(objects[i].transform.position);
            xs[i] = (double)screenPos.x;
            ys[i] = (double)screenPos.y;
        }
        var image = CaptureImage(POV);
        HeaderMsg msg = new HeaderMsg();
        msg.frame_id = "image_and_src_mat";
        ImageMsg imgmsg  = image.ToImageMsg(msg);
        ImageSrcMatMsg img_src_mat_msg  = new ImageSrcMatMsg();
        img_src_mat_msg.Image = imgmsg;
        img_src_mat_msg.x = xs;
        img_src_mat_msg.y = ys;
        ros.Publish(imageAndSrcmatTopic, img_src_mat_msg);
        Destroy(image);
    }

    public Texture2D CaptureImage(Camera cam){
        cam.backgroundColor = Color.black;
        cam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        cam.Render();
        // Texture2D pov_texture = new Texture2D(Pwdith, Pheight) ;
        Texture2D pov_texture = new Texture2D(Pwdith, Pheight, TextureFormat.RGB24, false) ;
        pov_texture.ReadPixels(new Rect(0, 0, Pwdith, Pheight), 0, 0);
        pov_texture.Apply();
        RenderTexture.active = currentRT;
        renderTexture.Release();
        cam.targetTexture = null;
        // Debug.Log(string.Format("format: {0}",pov_texture.format));

        return pov_texture;
    }

    void Update(){

        if (use_headtrack){
            // Vector3 trans    = new Vector3( htrans_x, htrans_y + 1.7f, 1.5f );
            Vector3 trans    = new Vector3(htrans_x, htrans_y + 0.35f,htrans_z + 0.20f);

            // Vector3 trans    = new Vector3(trans_x,trans_y,trans_z);
            // Quaternion quat  = new Quaternion(hquat_x,hquat_y,hquat_z,hquat_w);
            Quaternion quat  = new Quaternion(0,0,0,1);
            POV.transform.SetPositionAndRotation(trans,quat);
            // New_POV.transform.SetPositionAndRotation(trans,quat);
            // New_POV_1.transform.SetPositionAndRotation(trans,quat);
            Vector3 lookat_axis = new Vector3(0,1,0);
            POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
            // New_POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
            // New_POV_1.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        }
        else{

        // Vector3 trans    = new Vector3(trans_x -0.37f,trans_y + 0.40f,trans_z + 0.6f);
        Vector3 trans    = new Vector3(trans_x, trans_y + 0.35f,trans_z + 0.20f);
        // Vector3 trans    = new Vector3(trans_x,trans_y + 0.40f,1.0f + 0.6f);
        Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
        POV.transform.SetPositionAndRotation(trans,quat);
        // New_POV.transform.SetPositionAndRotation(trans,quat);
        // New_POV_1.transform.SetPositionAndRotation(trans,quat);
        Vector3 lookat_axis = new Vector3(0,1,0);
        POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        // New_POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        // New_POV_1.transform.LookAt(Cube_2_base.transform,lookat_axis); 

        }
        
        publish_imageNsrcmat();
        
        // Quaternion quat_base = Cube_2_base.transform.rotation;
        // Quaternion quat_pov  = POV.transform.rotation;

        // Vector3 euler = quat.eulerAngles;
        // Debug.Log(string.Format("x: {0}, y:{1}, z:{2}",euler[0],euler[1],euler[2]));

        // Quaternion rel_quat  = quat_pov * Quaternion.Inverse(quat_base);
        // Vector3 rel_trans    = POV.transform.position - Cube_2_base.transform.position;

        // CamPoseMsg pose  = new CamPoseMsg();
        // pose.x = rel_trans[0];
        // pose.y = rel_trans[1];
        // pose.z = rel_trans[2];

        // pose.x_ = rel_quat.x;
        // pose.y_ = rel_quat.y;
        // pose.z_ = rel_quat.z;
        // pose.w_ = rel_quat.w;
        // ros.Publish("pov_rel_pose", pose);


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