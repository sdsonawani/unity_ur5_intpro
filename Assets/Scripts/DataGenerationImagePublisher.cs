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

public class DataGenerationImagePublisher: MonoBehaviour{

    public Camera _camera; 
    ROSConnection ros;
    private ImageMsg imgmsg = new ImageMsg();
    private string CameraTopic1 = "top_down_camera";
    private float publishMessageFrequency = 0.01f;
    private float timeElapsed;
    private RenderTexture renderTexture;
    private RenderTexture renderTexturePlane;
    // public float f = 35.0f;
    private int Pwdith = 1920;
    private int Pheight = 1080;
    // public float f = 1.0f;
    // public float fov = 75f;
    // public string render_layer;
    private int id_counter = 1;

    public void Start(){

        _camera = GameObject.Find("Front_Camera").GetComponent<Camera>();
        ros =  ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(CameraTopic1);
        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UNorm);
        renderTexture.Create();
        _camera.clearFlags = CameraClearFlags.SolidColor;
    }


    public Texture2D CaptureImage(Camera cam){
        // cam.backgroundColor = Color.black;
        cam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        cam.Render();
        Texture2D pov_texture = new Texture2D(Pwdith, Pheight, TextureFormat.RGB24, false) ;
        pov_texture.ReadPixels(new Rect(0, 0, Pwdith, Pheight), 0, 0);
        pov_texture.Apply();
        RenderTexture.active = currentRT;
        cam.targetTexture = null;
        return pov_texture;
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

     public void save_locally(int id){
        var image    = CaptureImage(_camera);
        byte[] bytes = image.EncodeToPNG();
        File.WriteAllBytes(string.Format("/home/slocal/image_Data/Image_{0}.png",id), bytes);
        Destroy(image);
    }

    public void Update(){        
            Publish_1();
            // save_locally(id_counter);
            // id_counter += 1;
    }
}