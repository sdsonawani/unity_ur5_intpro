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


public class CamPoseSubscriberDataGeneration: MonoBehaviour{

    public bool use_headtrack = false;
    private GameObject Cube_1;
    private GameObject Cube_2;
    private GameObject Cube_2_base;
    private GameObject Cube_2_base1;
    private GameObject Cube_2_basec1;
    private GameObject Cube_2_basec2;
    private GameObject Cube_2_basec3;
    private GameObject Cube_2_basec4;

    private Camera POV = new Camera();
    private Camera New_POV = new Camera();
    // public Camera New_POV_1 = new Camera();

    public float table_x = 0.0f;
    public float table_y = 0.72f;
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
    private int Pwdith  = 1920;
    private int Pheight = 1080;

        
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

        // Cube_1        = GameObject.Find("Cube_1");
        // Cube_2        = GameObject.Find("Cube_2");
        Cube_2_base   = GameObject.Find("Cube_5");
        // Cube_2_base1   = GameObject.Find("Checkerboard");
        Cube_2_basec1 = GameObject.Find("Cube_1");
        Cube_2_basec2 = GameObject.Find("Cube_2");
        Cube_2_basec3 = GameObject.Find("Cube_3");
        Cube_2_basec4 = GameObject.Find("Cube_4");
        objects =  new List<GameObject> (){Cube_2_basec1, Cube_2_basec4, Cube_2_basec3, Cube_2_basec2 };
        shade = Shader.Find("Unlit/Color");
        Color customColor = new Color(0.4f, 0.9f, 0.7f, 1.0f);
        Color new_color   = new Color(1f, 1f, 1f, 1.0f);
        // ApplyMaterial(Cube_2_base   , new_color);
        // ApplyMaterial(Cube_2_basec1 , Color.red);
        // ApplyMaterial(Cube_2_basec2,  Color.green);
        // ApplyMaterial(Cube_2_basec3,  Color.blue);
        // ApplyMaterial(Cube_2_basec4,  Color.cyan);
        
    
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
        Vector3 cube2_base_trans = new Vector3(table_x + 0.01f, table_y, table_z - 0.04f);
        Vector3 cube2_base_trans1 = new Vector3(table_x , table_y+10.01f, table_z);
        Quaternion cube2_base_quat = Quaternion.Euler(0, 0, 0);
        ApplyPRAndCollision(Cube_2_base, cube2_base_trans, cube2_base_quat);
        // ApplyPRAndCollision(Cube_2_base1, cube2_base_trans1, cube2_base_quat);
        
        // RGB Planes
        // Vector3 cube2_base_c1_trans = new Vector3(-0.5f , table_y, 0.2f);
        // Vector3 cube2_base_c2_trans = new Vector3( 0.5f , table_y, 0.2f);
        // Vector3 cube2_base_c3_trans = new Vector3( 0.5f , table_y, 1.0f);    
        // Vector3 cube2_base_c4_trans = new Vector3(- 0.5f , table_y, 1.0f);    

        Vector3 cube2_base_c1_trans = new Vector3(- 0.74f, table_y, -0.04f);
        Vector3 cube2_base_c2_trans = new Vector3(  0.76f, table_y, -0.04f);
        Vector3 cube2_base_c3_trans = new Vector3(  0.76f, table_y, 0.74f);    
        Vector3 cube2_base_c4_trans = new Vector3(- 0.74f, table_y, 0.74f);
        ApplyPRAndCollision(Cube_2_basec1, cube2_base_c1_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec2, cube2_base_c2_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec3, cube2_base_c3_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec4, cube2_base_c4_trans, Quaternion.Euler(0,0,0));


        // Vector3 cube1_trans = new Vector3( (table_x + 0.2f) , (table_y + cube_height), table_z);
        // Quaternion cube1_quat = Quaternion.Euler(0, 0, 90f);
        // ApplyPRAndCollision(Cube_1, cube1_trans, cube1_quat);

        // Vector3 cube2_trans = new Vector3(-(table_x + 0.2f),  (table_y + cube_height), table_z);
        // Quaternion cube2_quat = Quaternion.Euler(0, 0, 0);
        // ApplyPRAndCollision(Cube_2, cube2_trans, cube2_quat);

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
            Vector3 trans    = new Vector3(htrans_x, htrans_y + 0.35f,htrans_z + 0.20f);
            Quaternion quat  = new Quaternion(0,0,0,1);
            POV.transform.SetPositionAndRotation(trans,quat);
            Vector3 lookat_axis = new Vector3(0,1,0);
            POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
        }
        else{

            Vector3 trans    = new Vector3(trans_x, trans_y + 0.72f, trans_z);
            Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
            POV.transform.SetPositionAndRotation(trans,quat);
            Vector3 lookat_axis = new Vector3(0,1,0);
            POV.transform.LookAt(Cube_2_base.transform,lookat_axis); 
            // Debug.Log(trans);

        }
        
        publish_imageNsrcmat();
    }
}