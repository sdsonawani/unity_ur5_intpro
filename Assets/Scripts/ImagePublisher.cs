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

public class ImagePublisher: MonoBehaviour{

    public Camera _camera; 

    ROSConnection ros;
    public ImageMsg imgmsg = new ImageMsg();
    public string CameraTopic = "pov_image";
    public HeaderMsg msg_ = new HeaderMsg();

    public float publishMessageFrequency = 0.01f;
    private float timeElapsed;
    public RenderTexture renderTexture;

    public void Start(){

        // Initialize ROS connection 
        ros =  ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(CameraTopic);
        _camera.enabled = true;

        // Render Texture Initialized
        renderTexture = new RenderTexture(_camera.pixelWidth, _camera.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
        _camera.clearFlags = CameraClearFlags.SolidColor;
    }

    public ImageMsg CaptureImage(){
        _camera.backgroundColor = Color.black;
        _camera.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        _camera.Render();
        Texture2D pov_texture = new Texture2D(renderTexture.width, renderTexture.height);
        pov_texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        pov_texture.Apply();
        RenderTexture.active = currentRT;
        _camera.targetTexture = null;
        // byte[] imageBytes = pov_texture.GetRawTextureData();
        msg_.frame_id = "pov_frame";
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
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency){
            ImageMsg rosmsg =  CaptureImage();
            ros.Publish(CameraTopic, rosmsg);
            timeElapsed = 0;
        }
    }
}