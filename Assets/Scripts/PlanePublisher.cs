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

public class PlanePublisher: MonoBehaviour{

    public Camera _2d_pov; 

    ROSConnection ros;
    public ImageMsg imgmsg = new ImageMsg();
    public string CameraTopic1 = "pov_plane";
    public HeaderMsg msg_ = new HeaderMsg();

    public float publishMessageFrequency = 0.01f;
    private float timeElapsed;
    public RenderTexture renderTexture;
    // public float f = 35.0f;
    public float f = 1.0f;
    public int Pwdith = 1920;
    public int Pheight = 1080;
    public float fov = 75f;
    public void Start(){

        _2d_pov = GameObject.Find("New_POV").GetComponent<Camera>();
        changeCameraParam(_2d_pov);
        
        // Initialize ROS connection 
        ros =  ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(CameraTopic1);
        // _camera.pixelWidth = 1920;
        // _camera.pixelHeight= 1080;
        // Render Texture Initialized
        // renderTexture = new RenderTexture(_camera.pixelWidth, _camera.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
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

    public void BgrToRgb(byte[] data) {
        for (int i = 0; i < data.Length; i += 3)
        {
            byte dummy = data[i];
            data[i] = data[i + 2];
            data[i + 2] = dummy;
        }
    }


    public void Update(){        
        // timeElapsed += Time.deltaTime;
        // if (timeElapsed > publishMessageFrequency){
            ImageMsg rosmsg1=  CaptureImage(_2d_pov, "pov_plane");
            ros.Publish(CameraTopic1, rosmsg1);
    }
}