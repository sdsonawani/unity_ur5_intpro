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


    private GameObject Cube_1;
    private GameObject Cube_2;
    private GameObject Cube_2_base;
    private GameObject Cube_2_base1;
    private GameObject Cube_2_basec1;
    private GameObject Cube_2_basec2;
    private GameObject Cube_2_basec3;
    private GameObject Cube_2_basec4;

    private Camera POV = new Camera();
    private int PwdithProj  = 1920;
    private int PheightProj = 1080;

    private List<GameObject> ref_objects;

    public float table_x = 0.0f;
    public float table_y = 0.72f;
    public float table_z = 0.47f;



    // private Camera _camera; 
    ROSConnection ros;
    private ImageMsg imgmsg = new ImageMsg();
    private string RosTopic = "custom_rgb_xy_src_mat";
    private string RosTopic1 = "unity_data_stream";
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
    private int split_factor = 8; // out of 10 objects, take first 4 objects for data generation
    private int max_objects = 3; // maximum number of objects on the table

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
    int data_counter =0 ;

    GameObject load_object(string object_name, int layer_id){
        GameObject gameobject = new OBJLoader().Load(string.Format("Assets/Resources/{0}.obj",object_name)); 
        gameobject.layer  = LayerMask.NameToLayer(string.Format("render_layer_{0}",1));
        
        GameObject child_object = Resources.Load<GameObject>(object_name).transform.GetChild(0).gameObject;

        child_object.transform.localScale = local_scale;
        gameobject.transform.localScale = local_scale;

        // Rigidbody body = gameobject.AddComponent<Rigidbody>();
        // body.angularDrag = 0.0f;
        // body.mass = 0.05f;
        // body.constraints = RigidbodyConstraints.FreezeRotationY;
        // body.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.constraints = RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.constraints = RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.useGravity = true;

        // MeshCollider mesh_ = gameobject.AddComponent<MeshCollider>();
        // var mesh = child_object.GetComponent<MeshFilter>().sharedMesh;
        // mesh_.sharedMesh = mesh;
        // mesh_.convex = true;
        // gameobject.AddComponent<BoxCollider>();
        if (object_name.Contains("hamburger")){
            gameobject.transform.localRotation = Quaternion.Euler(90, 0, 0);

        }
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

    // public Texture2D CaptureImage(Camera cam){
    //     // cam.backgroundColor = Color.black;
    //     cam.targetTexture = renderTexture;
    //     RenderTexture currentRT = RenderTexture.active;
    //     RenderTexture.active = renderTexture;
    //     cam.Render();
    //     Texture2D pov_texture = new Texture2D(Pwdith, Pheight, TextureFormat.RGB24, false) ;
    //     pov_texture.ReadPixels(new Rect(0, 0, Pwdith, Pheight), 0, 0);
    //     pov_texture.Apply();
    //     RenderTexture.active = currentRT;
    //     cam.targetTexture = null;
    //     return pov_texture;
    // }

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

    Tuple<List<GameObject>, List<string>> spawn_objects(){

        List<GameObject> loaded_objects = new List<GameObject>();
        List<string> current_object_names = new List<string>();
        HashSet<int> unique_object_ids = new  HashSet<int>();
        HashSet<int> unique_split_ids = new  HashSet<int>();
        HashSet<int> unique_xpos_ids = new  HashSet<int>();
        HashSet<int> unique_zpos_ids = new  HashSet<int>();

        for (int i = 0; i < max_objects; i++){
            int object_id = random.Next(0,data_object_names.Length);
            int split_id = random.Next(4,split_factor);

            while(unique_object_ids.Contains(object_id)){
                object_id = random.Next(0,data_object_names.Length);
            }
            while(unique_split_ids.Contains(split_id)){
                split_id = random.Next(4,split_factor);
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

            // Debug.Log(string.Format("unique ids: {0}|{1} and object: {2}", uoid, usid, object_name));
            GameObject local_object = load_object(object_name,1);
            float[] xz = grid[unique_xpos_ids_list[j],unique_zpos_ids_list[grid_z-1-j]];
            local_object.transform.position = new Vector3(xz[0], 0.77f, xz[1]);
            loaded_objects.Add(local_object);
            current_object_names.Add(object_name);
        }
        return new Tuple<List<GameObject>, List<string>>(loaded_objects,current_object_names);
    }

    
    // RGBXYSEGMsg get_CustomMsg(){
    
    //     var image    = CaptureImage(_camera);
    //     RGBXYSEGMsg custom_msg = new RGBXYSEGMsg();
    //     HeaderMsg msg = new HeaderMsg();
    //     msg.frame_id = RosTopic;
    //     ImageMsg imgmsg  = image.ToImageMsg(msg);
        
    //     int total_objects = current_gameobjects.Count + 1;
    //     float[] x_image = new float[total_objects];
    //     float[] y_image = new float[total_objects];
    //     float[] x_world = new float[total_objects];
    //     float[] y_world = new float[total_objects];
    //     float[] z_world = new float[total_objects];
    //     string[] object_names_msg = new string[total_objects];

    //     for (int i = 0; i < total_objects; i++){
    //         if (i == total_objects-1 ){
    //             Vector3 screenPos = _camera.WorldToScreenPoint(ur5_ee.transform.position);
    //             Vector3 relativePosition = baseUr5.transform.InverseTransformPoint(ur5_ee.transform.position);
    //             // Debug.Log(string.Format("relative position: {0}",relativePosition));
    //             // x_world[i] = objects[i].transform.position.x;
    //             // y_world[i] = objects[i].transform.position.y;
    //             // z_world[i] = objects[i].transform.position.z;
    //             x_world[i] = relativePosition.x;
    //             y_world[i] = relativePosition.y;
    //             z_world[i] = relativePosition.z;
    //             x_image[i] = screenPos.x;
    //             y_image[i] = screenPos.y;
    //             object_names_msg[i] = "ee_link";
    //         }
    //         else{
    //             Vector3 screenPos = _camera.WorldToScreenPoint(current_gameobjects[i].transform.position);
    //             Vector3 relativePosition = baseUr5.transform.InverseTransformPoint(current_gameobjects[i].transform.position);
    //             // Debug.Log(string.Format("relative position: {0}",relativePosition));
    //             // x_world[i] = objects[i].transform.position.x;
    //             // y_world[i] = objects[i].transform.position.y;
    //             // z_world[i] = objects[i].transform.position.z;
    //             x_world[i] = relativePosition.x;
    //             y_world[i] = relativePosition.y;
    //             z_world[i] = relativePosition.z;
    //             x_image[i] = screenPos.x;
    //             y_image[i] = screenPos.y;
    //             // Debug.Log(current_object_names_local[i]);
    //             object_names_msg[i] = current_object_names_local[i];
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
    //     custom_msg.data_counter = data_counter;
    //     // ros.Publish(RosTopic, custom_msg);

    //     Destroy(image);
    //     return custom_msg;
    // }

    // ImageSrcMatMsg get_imageNsrcmat(){
    //     // POV.cullingMask = LayerMask.GetMask("render_layer_1");
        
    //     double[] xs = new double[4];
    //     double[] ys = new double[4];
    //     for (int i = 0; i < 4; i++){
    //         Vector3 screenPos = POV.WorldToScreenPoint(ref_objects[i].transform.position);
    //         xs[i] = (double)screenPos.x;
    //         ys[i] = (double)screenPos.y;
    //     }
    //     var image = CaptureImageImgSrcMat(POV);
    //     HeaderMsg msg = new HeaderMsg();
    //     msg.frame_id = "image_and_src_mat";
    //     ImageMsg imgmsg  = image.ToImageMsg(msg);
    //     ImageSrcMatMsg img_src_mat_msg  = new ImageSrcMatMsg();
    //     img_src_mat_msg.Image = imgmsg;
    //     img_src_mat_msg.x = xs;
    //     img_src_mat_msg.y = ys;
    //     Destroy(image);
    //     return img_src_mat_msg;
    // }

    DataStreamMsg get_DataStreamMsg(){
        DataStreamMsg ros_msg = new DataStreamMsg();
        var image = CaptureImageImgSrcMat(POV);
        HeaderMsg header_msg = new HeaderMsg();
        header_msg.frame_id = "data_stream";
        ImageMsg imgmsg  = image.ToImageMsg(header_msg);
        int total_objects = current_gameobjects.Count + 1;
        float[] x_image = new float[total_objects];
        float[] y_image = new float[total_objects];
        float[] x_world = new float[total_objects];
        float[] y_world = new float[total_objects];
        float[] z_world = new float[total_objects];
        string[] object_names_msg = new string[total_objects];

        float[] xs = new float[4];
        float[] ys = new float[4];
        for (int i = 0; i < 4; i++){
            Vector3 screenPos = POV.WorldToScreenPoint(ref_objects[i].transform.position);
            xs[i] = (float)screenPos.x;
            ys[i] = (float)screenPos.y;
        }

        for (int i = 0; i < total_objects; i++){
            if (i == total_objects-1 ){
                Vector3 screenPos = POV.WorldToScreenPoint(ur5_ee.transform.position);
                Vector3 relativePosition = baseUr5.transform.InverseTransformPoint(ur5_ee.transform.position);
                x_world[i] = relativePosition.x;
                y_world[i] = relativePosition.y;
                z_world[i] = relativePosition.z;
                x_image[i] = screenPos.x;
                y_image[i] = screenPos.y;
                object_names_msg[i] = "ee_link";
            }
            else{
                Vector3 screenPos = POV.WorldToScreenPoint(current_gameobjects[i].transform.position);
                Vector3 relativePosition = baseUr5.transform.InverseTransformPoint(current_gameobjects[i].transform.position);
                x_world[i] = relativePosition.x;
                y_world[i] = relativePosition.y;
                z_world[i] = relativePosition.z;
                x_image[i] = screenPos.x;
                y_image[i] = screenPos.y;
                object_names_msg[i] = current_object_names_local[i];
            }
        }

        float[] joints= new float[7];
        for(var i = 0; i<6; i++ ){

                if (i==1 || i == 3){
                   joints[i] =  (float) (m_jab[i].GetPosition() - (Math.PI/2.0));
                }
                else{
                    joints[i] = (float) (m_jab[i].GetPosition());
                }
            }
        joints[^1] = 0.0f;

        ros_msg.Image = imgmsg;
        ros_msg.x = xs;
        ros_msg.y = ys;
        ros_msg.joint_angles = joints;
        ros_msg.x_image = x_image;
        ros_msg.y_image = y_image;
        ros_msg.x_world = x_world;
        ros_msg.y_world = y_world;
        ros_msg.z_world = z_world;
        ros_msg.obj_names = object_names_msg;
        ros_msg.data_counter = data_counter;
        image = null;
        return ros_msg;
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

        Cube_2_base   = GameObject.Find("Cube_5");
        Cube_2_basec1 = GameObject.Find("Cube_1");
        Cube_2_basec2 = GameObject.Find("Cube_2");
        Cube_2_basec3 = GameObject.Find("Cube_3");
        Cube_2_basec4 = GameObject.Find("Cube_4");
        ref_objects =  new List<GameObject> (){Cube_2_basec1, Cube_2_basec4, Cube_2_basec3, Cube_2_basec2 };

        Vector3 cube2_base_trans = new Vector3(table_x + 0.01f, table_y, table_z - 0.04f);
        Quaternion cube2_base_quat = Quaternion.Euler(0, 0, 0);
        ApplyPRAndCollision(Cube_2_base, cube2_base_trans, cube2_base_quat);   

        Vector3 cube2_base_c1_trans = new Vector3(- 0.74f, table_y, -0.04f);
        Vector3 cube2_base_c2_trans = new Vector3(  0.76f, table_y, -0.04f);
        Vector3 cube2_base_c3_trans = new Vector3(  0.76f, table_y, 0.74f);    
        Vector3 cube2_base_c4_trans = new Vector3(- 0.74f, table_y, 0.74f);
        ApplyPRAndCollision(Cube_2_basec1, cube2_base_c1_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec2, cube2_base_c2_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec3, cube2_base_c3_trans, Quaternion.Euler(0,0,0));
        ApplyPRAndCollision(Cube_2_basec4, cube2_base_c4_trans, Quaternion.Euler(0,0,0));

        POV       = GameObject.Find("POV").GetComponent<Camera>();
        // PwdithProj  = POV.pixelWidth;
        // PheightProj = POV.pixelHeight;
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
        ros.RegisterPublisher<RGBXYImageSrcMatMsg>(RosTopic);
        ros.RegisterPublisher<DataStreamMsg>(RosTopic1);
        // Pwdith  = _camera.pixelWidth;
        // Pheight = _camera.pixelHeight;

        renderTexture = new RenderTexture(Pwdith, Pheight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8_UNorm);
        renderTexture.Create();


        // intiate object 
        object_names = get_object_names(resource_dir);
        // Debug.Log(string.Format("total objects found in resources: {0}",data_object_names.Length));
    }

    void Update() {

        time_elapsed += Time.deltaTime;
    
        if (time_elapsed > (float) (1/freq)){
            if (initiate){
                Tuple<List<GameObject>, List<string>> current_data = spawn_objects();
                current_gameobjects = current_data.Item1;
                current_object_names_local = current_data.Item2;

                var ros_msg = get_DataStreamMsg();
                ros.Publish(RosTopic1,ros_msg);
                // ImageSrcMatMsg msg1 = get_imageNsrcmat();
                // RGBXYSEGMsg msg2    = get_CustomMsg();
                // RGBXYImageSrcMatMsg msg = new RGBXYImageSrcMatMsg();
                // msg.imagsrcmat = msg1;
                // msg.rgbxyseg = msg2;
                // ros.Publish(RosTopic,msg);
                
                initiate = false;
            }
            else{
                // Debug.Log("destroying objects");
                for (var i  = 0; i < current_gameobjects.Count; i ++){
                    Destroy(current_gameobjects[i]);
                    }
                current_gameobjects = new List<GameObject>();
                current_object_names_local = new List<string>();
                initiate = true;
                data_counter += 1;
            }
            time_elapsed = 0;

        }
    }
}
