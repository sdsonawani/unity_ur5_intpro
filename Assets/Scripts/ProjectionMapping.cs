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



class ProjectionMapping: MonoBehaviour{
    public Camera LookUpCam;
    GameObject LookUpCamObj;

    
    public RenderTexture renderTexture;


    ROSConnection ros;
    public string CameraTopic = "shadow_image";


    void Start(){
        LookUpCamObj = GameObject.Find("LookUp");
        LookUpCam = LookUpCamObj.GetComponent<Camera>();

        ros =  ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(CameraTopic);

        renderTexture = new RenderTexture(LookUpCam.pixelWidth, 
                                          LookUpCam.pixelHeight, 
                                          24, 
                                          UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
    }

    // Render Texture from LookUp camera placed near ur5_1
    public Texture2D RenderShadow(){
        LookUpCam.backgroundColor = Color.black;
        LookUpCam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        LookUpCam.Render();
        Texture2D ShadowImage = new Texture2D(renderTexture.width,
                                         renderTexture.height);
        ShadowImage.ReadPixels(new Rect(0, 
                                   0, 
                                   renderTexture.width, 
                                   renderTexture.height), 0, 0);
        RenderTexture.active = currentRT;
        LookUpCam.targetTexture = null;
        return ShadowImage;
    }


    void Update(){
        
        Texture2D ShadowImage = RenderShadow();

        // ProjSurfMaterial = ProjSurf.GetComponent<Renderer>().material;
        // ProjSurfMaterial.SetTexture("_MainTex",testText);
        // ProjSurfMaterial.SetTexture("_MainTex",ShadowImage);
        // ProjSurfMaterial.color = Color.red;
        // ProjSurfMaterial.mainTexture = ShadowImage;
        // rend.material.mainTexture = ShadowImage;
        // rend.material.SetTexture("_MainTex",ShadowImage);
        // rend.material.color = Color.red;    
        // rend.material.color.a = 0;    
        ShadowImage.Apply();

        


        // header frame id information for ros msg
        HeaderMsg hmsg   = new HeaderMsg();
        hmsg.frame_id = "shadow_frame";
        
        // converting texture 2d to ros img msg
        ImageMsg  imgmsg = new ImageMsg(); 
        imgmsg = ShadowImage.ToImageMsg(hmsg);
        ros.Publish(CameraTopic,imgmsg);
        
    
        Destroy(ShadowImage);
        // Debug.Log("Update in Projection Mapping");

    }
}