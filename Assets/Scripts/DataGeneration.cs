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

public class DataGeneration: MonoBehaviour{


    private Camera _camera; 
    ROSConnection ros;
    private ImageMsg imgmsg = new ImageMsg();
    private string CameraTopic1 = "custom_rgb_objs_xy";
    private float timeElapsed;
    private RenderTexture renderTexture;
    private RenderTexture renderTexturePlane;
    private int Pwdith = 1920;
    private int Pheight = 1080;

    public string[,] object_names;
    public List<int> unique_object_ids_list;
    public List<int> unique_split_ids_list;
    public List<int> unique_xpos_ids_list;
    public List<int> unique_zpos_ids_list;
    public List<GameObject> loaded_objects;
    public List<int> obj_ids;

    private Vector3 local_scale = new Vector3(0.05f,0.05f,0.05f); 
    private string resource_dir = string.Format("/home/slocal/Documents/ur5_intpro/Assets/Resources/");
    UrdfJointRevolute[] m_jab;
    private static readonly string[] LinkNames =
        { "world/dummy_link/base_link/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link",  "/wrist_2_link",  "/wrist_3_link" };
    private string ee_link1 = "world/dummy_link/base_link/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link";
    private string ee_link2 = "/tool0/robotiq_coupler/robotiq_85_base_link";
    private GameObject ur5;
    private GameObject ur5_ee;
    private GameObject baseUr5;
    private SerializedObject tagManager;
    private SerializedProperty tagsProp;

    private static readonly string[] data_object_names = {"apple","avocado", "donut", "green pear", "hamburger", "lemon", "mug", "orange", 
                                                          "sourdough", "strawberry"};    
    private int _counter = 0;
    private int split_factor = 4; // out of 10 objects, take first 4 objects for data generation
    private int max_objects = 3; // maximum number of objects on the table

    public float x_max = 0.5f;
    public float x_min = -0.5f;

    public float z_max = 0.5f;
    public float z_min = 0.2f;

    public int grid_x = 3;
    public int grid_z = 3; 

    Random random = new Random();
    GameObject load_object(string object_name, int layer_id){
        GameObject gameobject = new OBJLoader().Load(string.Format("Assets/Resources/{0}.obj",object_name)); 
        gameobject.layer  = LayerMask.NameToLayer(string.Format("render_layer_{0}",1));
        
        GameObject child_object = Resources.Load<GameObject>(object_name).transform.GetChild(0).gameObject;

        child_object.transform.localScale = local_scale;
        gameobject.transform.localScale = local_scale;

        Rigidbody body = gameobject.AddComponent<Rigidbody>();
        body.angularDrag = 0.0f;
        body.mass = 0.05f;
        // body.constraints = RigidbodyConstraints.FreezeRotationY;
        // body.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.constraints = RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.constraints = RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        body.useGravity = true;

        // MeshCollider mesh_ = gameobject.AddComponent<MeshCollider>();
        // var mesh = child_object.GetComponent<MeshFilter>().sharedMesh;
        // mesh_.sharedMesh = mesh;
        // mesh_.convex = true;
        gameobject.AddComponent<BoxCollider>();
        gameobject.transform.localRotation = Quaternion.Euler(90, 0, 0);
        gameobject.transform.GetChild(0).gameObject.transform.localRotation = Quaternion.Euler(180, 180, 0);
        gameobject.transform.GetChild(0).gameObject.layer  = LayerMask.NameToLayer(string.Format("render_layer_{0}",1));

        // change the material properties
        var Renderer =  gameobject.transform.GetChild(0).gameObject.GetComponent<Renderer>();
        Renderer.material.SetColor("_SpecColor",Color.black);
        
        // add render layer id to object
        // gameobject.transform.GetChild(0).gameObject.layer = LayerMask.NameToLayer(string.Format("render_layer_{0}",layer_id));
        // gameobject.layer = LayerMask.NameToLayer(string.Format("render_layer_{0}",layer_id));
        // gameobject.tag = tag_id;
        return gameobject;
    }

    string[,] get_object_names(string resource_dir){
        string[,] object_names_per_id = new string[data_object_names.Length,10];
        var flag = Directory.Exists(resource_dir);
        // Debug.Log(string.Format("checking if directory exist: {0}",flag));
        if (!flag){
            throw new Exception("Resource directory does not exist...");
        }
        string[] files = Directory.GetFiles(resource_dir);
        for (var i = 0; i < data_object_names.Length; i++){
            int y_counter = 0;

            for (var j = 0; j < files.Length; j++){
                string[] split_string = files[j].Split('.');
                var file_format = split_string[^1];

                if (file_format == "obj"){
                    var object_name = files[j].Split('/')[^1].Split('.')[0];
                    // Debug.Log(string.Format("object name: {0} and data name: {1}", object_name.Split("_")[0], data_object_names[i]));
                    // Debug.Log(string.Format("object_name: {0} and data_object_name: {1} and is exist: {2}", object_name, data_object_names[i],object_name.Contains(data_object_names[i])));
                    // if (data_object_names[i] == object_name.Split("_")[0]){
                    if (object_name.Contains(data_object_names[i])){
                        // Debug.Log(object_name.Split("_")[0] + "_" + y_counter.ToString() );

                        object_names_per_id[i,y_counter] = object_name;
                        // Debug.Log(object_names_per_id[i,y_counter]);
                        y_counter += 1;
                        if (y_counter == 10){
                            break;
                        }
                    }
                }
            }
        }
        
        return object_names_per_id;
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

    // public void PublishDataMsg(){
    //     string topic_name = CameraTopic1;
    //     var image    = CaptureImage(_camera);
    //     RGBXYSEGMsg custom_msg = new RGBXYSEGMsg();
    //     HeaderMsg msg = new HeaderMsg();
    //     msg.frame_id = topic_name;
    //     ImageMsg imgmsg  = image.ToImageMsg(msg);
        
    //     int total_objects = objects.Count;
    //     float[] x_image = new float[total_objects];
    //     float[] y_image = new float[total_objects];
    //     float[] x_world = new float[total_objects];
    //     float[] y_world = new float[total_objects];
    //     float[] z_world = new float[total_objects];
    //     string[] object_names_msg = new string[total_objects];

    //     for (int i = 0; i < objects.Count; i++){
    //         Vector3 screenPos = _camera.WorldToScreenPoint(objects[i].transform.position);
    //         Vector3 relativePosition = baseUr5.transform.InverseTransformPoint(objects[i].transform.position);
    //         // Debug.Log(string.Format("relative position: {0}",relativePosition));
    //         // x_world[i] = objects[i].transform.position.x;
    //         // y_world[i] = objects[i].transform.position.y;
    //         // z_world[i] = objects[i].transform.position.z;
    //         x_world[i] = relativePosition.x;
    //         y_world[i] = relativePosition.y;
    //         z_world[i] = relativePosition.z;
    //         x_image[i] = screenPos.x;
    //         y_image[i] = screenPos.y;

    //         if (i == objects.Count-1 ){
    //             object_names_msg[i] = "ee_link";
    //         }
    //         else{
    //             object_names_msg[i] = object_names[obj_ids[i]];
    //         }
    //     }
    //     // Vector3 screenPos_ee = _camera.WorldToScreenPoint(ur5.transform.Find(ee_link1+ee_link2).position);
    //     // Debug.Log(string.Format("ee: x = [{0}] y = [{1}]",screenPos_ee.x,screenPos_ee.y));
    //     // Debug.Log(ur5_ee.transform.position);
    //     float[] joints= new float[7];
    //     for(var i = 0; i<6; i++ ){

    //             if (i==1 || i == 3){
    //                joints[i] =  (float) (m_jab[i].GetPosition() - (Math.PI/2.0));
    //             }
    //             else{
    //                 joints[i] = (float) (m_jab[i].GetPosition());
    //             }
    //         }
    //     joints[^1] = 0.0f;
        
    //     custom_msg.Image = imgmsg;
    //     custom_msg.joint_angles = joints;
    //     custom_msg.x_image = x_image;
    //     custom_msg.y_image = y_image;
    //     custom_msg.x_world = x_world;
    //     custom_msg.y_world = y_world;
    //     custom_msg.z_world = z_world;
    //     custom_msg.obj_names = object_names_msg;

    //     ros.Publish(topic_name, custom_msg);

    //     Destroy(image);
    // }


    float[,][] GetPositionGrid(){
        float[,][] grid = new float[grid_x,grid_z][];
        float dx = (float) (x_max - x_min)/ grid_x;
        float dz = (float) (z_max - z_min)/ grid_z;
        Debug.Log(string.Format("dx: {0} and dz: {1}", dx, dz));
        for (var i = 0 ; i < grid_x; i++){
            float x_ = (float) (x_min + (i * dx));

            for (var j = 0 ; j < grid_z; j++){
                float y_ = (float) (z_min + (j * dz));
                Debug.Log(string.Format("x: {0}, z: {1}",x_, y_));
                float [] xz = new float[] {x_,y_};
                grid[i,j] = xz;
            }
        }
        return grid;
    }

    void Start(){
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
        _camera = GameObject.Find("Front_Camera_1").GetComponent<Camera>();
        ros =  ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RGBXYSEGMsg>(CameraTopic1);
        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UNorm);
        renderTexture.Create();


        // intiate object 
        object_names = get_object_names(resource_dir);
        // Debug.Log(string.Format("total objects found in resources: {0}",data_object_names.Length));
        HashSet<int> unique_object_ids = new  HashSet<int>();
        HashSet<int> unique_split_ids = new  HashSet<int>();
        HashSet<int> unique_xpos_ids = new  HashSet<int>();
        HashSet<int> unique_zpos_ids = new  HashSet<int>();

        for (int i = 0; i < max_objects; i++){
            int object_id = random.Next(0,data_object_names.Length);
            int split_id = random.Next(0,split_factor);

            while(unique_object_ids.Contains(object_id)){
                object_id = random.Next(0,data_object_names.Length);
            }
            while(unique_split_ids.Contains(split_id)){
                split_id = random.Next(0,split_factor);
            }
            unique_split_ids.Add(split_id);
            unique_object_ids.Add(object_id);
        }
        
        for (int i = 0; i < grid_x; i++){
            int x_id = random.Next(0,grid_x);
            while(unique_xpos_ids.Contains(x_id)){
                x_id = random.Next(0,grid_x);
            }
            unique_xpos_ids.Add(x_id);
        }

        for (int i = 0; i < grid_z; i++){
            int z_id = random.Next(0,grid_z);
            while(unique_zpos_ids.Contains(z_id)){
                z_id = random.Next(0,grid_z);
            }
            Debug.Log(z_id);
            unique_zpos_ids.Add(z_id);
        }

        unique_object_ids_list = new List<int>(unique_object_ids);
        unique_split_ids_list  = new List<int>(unique_split_ids);
        unique_xpos_ids_list   = new List<int>(unique_xpos_ids);
        unique_zpos_ids_list   = new List<int>(unique_zpos_ids);
        float[,][] grid = GetPositionGrid();

        for (int j = 0; j < max_objects; j++){
            
            int uoid = unique_object_ids_list[j];
            int usid = unique_split_ids_list[j];
            var object_name = object_names[unique_object_ids_list[j],unique_split_ids_list[j]];

            Debug.Log(string.Format("unique ids: {0}|{1} and object: {2}", uoid, usid, object_name));
            GameObject local_object = load_object(object_name,1);
            float[] xz = grid[unique_xpos_ids_list[j],unique_zpos_ids_list[grid_z-1-j]];
            local_object.transform.position = new Vector3(xz[0], 0.75f, xz[1]);
            loaded_objects.Add(local_object);
        }
        
    }

    // void Update() {
    // }

}
