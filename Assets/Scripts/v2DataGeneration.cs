using UnityEngine;
using UnityEngine.UI;
using UnityEditor;
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
using System.IO;
using Random=System.Random;
using Dummiesman;

public class v2DataGeneration: MonoBehaviour{


    private Camera POV = new Camera();
    private int PwdithProj  = 1920;
    private int PheightProj = 1080;

    // private Camera _camera; 
    UrdfJointRevolute[] m_jab;

    ROSConnection ros;
    private string RosTopic1 = "unity_data_stream_v2";
    private float timeElapsed;
    private RenderTexture renderTexture;
    private RenderTexture renderTexturePlane;
    private int Pwdith = 1920;
    private int Pheight = 1080;

    private static readonly string[] LinkNames =
        { "world/dummy_link/base_link/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link",  "/wrist_2_link",  "/wrist_3_link" };
    private string ee_link1 = "world/dummy_link/base_link/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link";
    private string ee_link2 = "/tool0/robotiq_coupler/robotiq_85_base_link";
    private GameObject ur5;
    private GameObject ur5_ee;
    private GameObject baseUr5;

    public float x_max = 0.5f;
    public float x_min = -0.5f;

    public float z_max = 0.5f;
    public float z_min = 0.2f;

    public int grid_x = 3;
    public int grid_z = 3; 

    public float freq = 1f;
    public float time_elapsed = 0;
    public float time_elapsed_destroy = 0;
    private bool initiate = true;

    List<GameObject> current_gameobjects;
    List<string> current_object_names_local;
    Random random = new Random();
    int data_counter = 0;

    public Texture2D CaptureImageImgSrcMat(Camera cam){
        cam.backgroundColor = Color.black;
        cam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        cam.Render();
        // Texture2D pov_texture = new Texture2D(PwdithProj, PheightProj) ;
        Texture2D pov_texture = new Texture2D(PwdithProj, PheightProj, TextureFormat.RGB24, false) ;
        pov_texture.ReadPixels(new Rect(0, 0, PwdithProj, PheightProj), 0, 0);
        pov_texture.Apply();
        RenderTexture.active = currentRT;
        renderTexture.Release();
        cam.targetTexture = null;
        // Debug.Log(string.Format("format: {0}",pov_texture.format));
        return pov_texture;
    }


    float[,][] GetPositionGrid(){
        float[,][] grid = new float[grid_x,grid_z][];
        float dx = (float) (x_max - x_min)/ grid_x;
        float dz = (float) (z_max - z_min)/ grid_z;
        // Debug.Log(string.Format("dx: {0} and dz: {1}", dx, dz));
        for (var i = 0 ; i < grid_x; i++){
            float x_ = (float) (x_min + (i * dx));

            for (var j = 0 ; j < grid_z; j++){
                float y_ = (float) (z_min + (j * dz));
                // Debug.Log(string.Format("x: {0}, z: {1}",x_, y_));
                float [] xz = new float[] {x_,y_};
                grid[i,j] = xz;
            }
        }
        return grid;
    }



    void Start(){

        POV       = GameObject.Find("POV").GetComponent<Camera>();
        POV.clearFlags = CameraClearFlags.SolidColor;

        // initiate ur5 variables
        ur5 = GameObject.Find("ur5");
        ur5_ee = GameObject.Find("Cube_ee");
        baseUr5 = GameObject.Find("base");
        m_jab = new UrdfJointRevolute[6];
        var link_names = string.Empty;
        for (var i = 0; i < 6; i++){
            link_names += LinkNames[i];
            m_jab[i] = ur5.transform.Find(link_names).GetComponent<UrdfJointRevolute>();
        }

        // initiate camera variables
        // _camera = GameObject.Find("Front_Camera_1").GetComponent<Camera>();
        ros =  ROSConnection.GetOrCreateInstance();
        // ros.RegisterPublisher<RGBXYImageSrcMatMsg>(RosTopic);

        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UNorm);
        renderTexture.Create();


    }

    void Update() {

        time_elapsed += Time.deltaTime;
    
        if (time_elapsed > (float) (1/freq)){
            Debug.Log("Updating....");
            time_elapsed = 0;
        }
    }
}
