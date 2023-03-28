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
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Threading;

public class ImagePublisher: MonoBehaviour{

    public Camera _camera; 
    public Camera _camera_r; 
    // public Camera _2d_pov; 
    // public Camera _2d_pov_1; 

    public GameObject target;
    ROSConnection ros;
    public ImageMsg imgmsg = new ImageMsg();
    public CamPoseMsg posemsg = new CamPoseMsg();

    public string CameraTopic1 = "pov_image";
    public string CamPoseTopic = "pov_unity_pose";
    // public string CameraTopic2 = "pov_plane";
    public string CameraTopic3 = "rgb_planes";
    public string CameraTopic4 = "pov_image_right";
    public HeaderMsg msg_ = new HeaderMsg();

    public float publishMessageFrequency = 0.01f;
    private float timeElapsed;
    public RenderTexture renderTexture;
    public RenderTexture renderTexturePlane;
    // public float f = 35.0f;
    public float f = 1.0f;
    public int Pwdith = 1920;
    public int Pheight = 1080;
    public float fov = 75f;
    public string render_layer;





    public void Start(){

        _camera = GameObject.Find("POV").GetComponent<Camera>();
        changeCameraParam(_camera);
        Pwdith  = _camera.pixelWidth;
        Pheight = _camera.pixelHeight;
        // target  =  GameObject.Find("Base_Plate");
        target  =  GameObject.Find("Base_Plate_c1");

        // _camera_r = GameObject.Find("POV_1").GetComponent<Camera>();
        // _camera_r.transform.SetParent(_camera.transform);
        // Vector3 shift_r  = new Vector3(0.05f, 0.0f, 0.0f);
        // _camera_r.transform.SetLocalPositionAndRotation(shift_r,Quaternion.Euler(0,0,0));
        // _2d_pov = GameObject.Find("New_POV").GetComponent<Camera>();
        // _2d_pov_1 = GameObject.Find("New_POV_1").GetComponent<Camera>();
        // changeCameraParam(_camera_r);
        // changeCameraParam(_2d_pov);
        // changeCameraParam(_2d_pov_1);
        
        // Initialize ROS connection 
        ros =  ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(CameraTopic1);
        // ros.RegisterPublisher<ImageMsg>(CameraTopic2);
        ros.RegisterPublisher<ImageMsg>(CameraTopic3);
        ros.RegisterPublisher<ImageMsg>(CameraTopic4);
        ros.RegisterPublisher<CamPoseMsg>(CamPoseTopic);
        // _camera.pixelWidth = 1920;
        // _camera.pixelHeight= 1080;
        // Render Texture Initialized
        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        // renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        // renderTexture      = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UInt);
        renderTexture.Create();
        _camera.clearFlags = CameraClearFlags.SolidColor;
    }


    public void changeCameraParam(Camera c_camera)
    {
        // height: 600
        // width: 800
        // distortion_model: "plumb_bob"
        // D: [1e-08, 1e-08, 1e-08, 1e-08, 1e-08]
        // K: [565.9971898668298, 0.0, 400.5, 0.0, 565.9971898668298, 300.5, 0.0, 0.0, 1.0]
        // R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        // P: [565.9971898668298, 0.0, 400.5, -0.0, 0.0, 565.9971898668298, 300.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        // binning_x: 0
        // binning_y: 0
        // roi: 
        // x_offset: 0
        // y_offset: 0
        // height: 0
        // width: 0
        // do_rectify: False
        float ax, ay, sizeX, sizeY;
        float x0, y0, shiftX, shiftY;
        int width, height;
 
 
        ax = 565.9971898668298f;
        ay = 565.9971898668298f;
        x0 = 400.5f;
        y0 = 300.5f;
 
        width =  Pwdith;
        height = Pheight;       

        sizeX = f * width / ax;
        sizeY = f * height / ay;
 
        //PlayerSettings.defaultScreenWidth = width;
        //PlayerSettings.defaultScreenHeight = height;
 
        // shiftX = -(x0 - width / 2.0f) / width;
        // shiftY = (y0 - height / 2.0f) / height;
        shiftX = 0.0f;
        shiftY = 0.0f;
 
        // c_camera.sensorSize = new Vector2(sizeX, sizeY);     // in mm, mx = 1000/x, my = 1000/y
        c_camera.sensorSize = new Vector2(1.0f, 1.0f);     // in mm, mx = 1000/x, my = 1000/y
        // c_camera.focalLength = 0.45f;                            // in mm, ax = f * mx, ay = f * my
        var fl = Camera.FieldOfViewToFocalLength(fov, 1.0f);          
        c_camera.focalLength =  fl;               // in mm, ax = f * mx, ay = f * my
        c_camera.lensShift = new Vector2(shiftX, shiftY);    // W/2,H/w for (0,0), 1.0 shift in full W/H in image plane
 
    }

    public ImageMsg CaptureImage(Camera cam, string fid){
        cam.backgroundColor = Color.black;
        cam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        cam.Render();
        // Texture2D pov_texture = new Texture2D(renderTexture.width, renderTexture.height);
        // pov_texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        Texture2D pov_texture = new Texture2D(Pwdith, Pheight);
        pov_texture.ReadPixels(new Rect(0, 0, Pwdith, Pheight), 0, 0);
        pov_texture.Apply();
        RenderTexture.active = currentRT;
        cam.targetTexture = null;
        // byte[] imageBytes = pov_texture.GetRawTextureData();
        msg_.frame_id = fid;
        ImageMsg imagemsg = pov_texture.ToImageMsg(msg_);
        Destroy(pov_texture);
        return imagemsg;
    }


    public Texture2D CaptureImage(Camera cam){
        cam.backgroundColor = Color.black;
        cam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        cam.Render();
        Texture2D pov_texture = new Texture2D(Pwdith, Pheight);
        pov_texture.ReadPixels(new Rect(0, 0, Pwdith, Pheight), 0, 0);
        pov_texture.Apply();
        RenderTexture.active = currentRT;
        cam.targetTexture = null;
        return pov_texture;
    }

    public List<int> cropImage(Texture2D image){
        
        List<int> cord = new List<int> ();
        // cord.Add(1);
        // cord.Add(2);
        int x_min = Pwdith;
        int x_max = 0;
        int y_min = 0;
        int y_max = Pheight;
        for (int i = 0; i < Pheight; i++){
            for (int j = 0; j< Pwdith; j++){
                var color = image.GetPixel(i,j);
                if (color.r == 1.0f){
                   if (x_min > j){
                    x_min = j;
                    break;
                   }
                   else if(x_max < j){
                    x_max = j;
                    break;

                   }
                }
            }
        }
        cord.Add(x_min);
        cord.Add(x_max);
        return cord;
    }

    public void Publish_1(){
        string topic_name = CameraTopic1;
        var image    = CaptureImage(_camera);
        HeaderMsg msg = new HeaderMsg();
        msg.frame_id = topic_name;
        ImageMsg imgmsg  = image.ToImageMsg(msg);
        ros.Publish(topic_name, imgmsg);
        Destroy(image);
    }

    public void Publish_2(){
        string topic_name = CameraTopic3;
        var image    = CaptureImage(_camera);
        Debug.Log(string.Format("texture dimension: {0}",image.width));

        HeaderMsg msg = new HeaderMsg();
        msg.frame_id = topic_name;
        ImageMsg imgmsg  = image.ToImageMsg(msg);


        ros.Publish(topic_name, imgmsg);
        Destroy(image);
    }

    public void Publish_3(){
        string topic_name = CameraTopic4;
        var image    = CaptureImage(_camera_r);
        HeaderMsg msg = new HeaderMsg();
        msg.frame_id = topic_name;
        ImageMsg imgmsg  = image.ToImageMsg(msg);
        ros.Publish(topic_name, imgmsg);
        Destroy(image);
    }


    public void Update(){        

            _camera.cullingMask = LayerMask.GetMask("render_layer_1");
            Vector3 screenPos = _camera.WorldToScreenPoint(target.transform.position);
            // Debug.Log("target is " + screenPos.x + " pixels from the left");
            Debug.Log(string.Format("x: {0}, y: {1}, z: {2}",screenPos.x,screenPos.y,screenPos.z));
            Publish_1();

            _camera.cullingMask = LayerMask.GetMask("render_layer_3");
            Publish_2();

            // _camera_r.cullingMask = LayerMask.GetMask("render_layer_1");
            // Publish_3();

            
            Vector3 trans = _camera.transform.position;
            // Quaternion quat = _camera.transform.rotation;
            posemsg.x = trans[0];
            posemsg.y = trans[1];
            posemsg.z = trans[2];
            ros.Publish(CamPoseTopic,posemsg);
    }
}