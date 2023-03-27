using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter.Control;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;
using System.Collections;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Threading;





public class Teleop: MonoBehaviour{
    public GameObject Cube1 ;
    public GameObject Cube2 ;
    public GameObject Cube3 ;
    public GameObject Cube4 ;
    public GameObject Cube5 ;

    public GameObject Cube1_shadow ;
    public GameObject Cube2_shadow ;
    public GameObject Cube3_shadow ;
    public GameObject Cube4_shadow ;
    public GameObject Cube5_shadow ;

    public GameObject Cube1_1 ;
    public GameObject Cube2_1 ;
    public GameObject Cube3_1 ;
    public GameObject Cube4_1 ;
    public GameObject Cube5_1 ;
    public GameObject HandL ;
    private List<GameObject> Objects;
    private List<GameObject> Objects1;

    ROSConnection ros;
    public string teleopTopicName = "obj_pose";
    public string swteleopTopicName = "smartwatch_pose";
    public string smteleopTopicName = "spacemouse_pose";
    public string primitveTopicName = "primitive_id";
    public string unityPrimitiveName = "unity_primitive_id";

    public bool use_opti = false;
    public bool use_smartwatch = false;
    public bool use_spacemouse = true;
    public bool use_speech2text = false;

    // public Vector3    init_trans = new Vector3(0.45f, 0.35f, 0.30f);
    private Vector3    init_trans = new Vector3(0.25f, 0.37f, 0.50f);
    private Vector3    init_trans1 = new Vector3(0.25f, 0.32f, 0.50f);
    private Quaternion init_quat  = new Quaternion(0,0,0,1);

    public Vector3    FrozenTrans = new Vector3(0.45f, 0.35f, 0.30f);
    public Quaternion FrozenQuat  = Quaternion.Euler(0,-90,0);

    public int obj_id  = 1;
    public int prim_id_max = 5;
    public int prim_id_min = 1;
    public int primitive_id;
    public int select_obj_id = 100;
    public int tmp = 100;

    public float hover_disp = 0.35f;
    public float hover_disp_max = 0.40f;

    private List<string> primitives = new List<string> (){"init","hover", "select", "translate", "rotate", "release"}; 
    private StringMsg msg = new StringMsg();
     
    private float trans_x, trans_y, trans_z;
    private float quat_w, quat_x, quat_y, quat_z;
    private float swtrans_x,swtrans_y,swtrans_z;
    private float swquat_w, swquat_x, swquat_y, swquat_z;

    private float xoffset  = 0.0f;
    private float yoffset  = 0.35f;
    private float zoffset  = 0.50f;

    private float smtrans_x = 0.0f;
    private float smtrans_y = 0.45f;
    private float smtrans_z = 0.50f;
    private float smroll  =  0.0f; 
    private float smpitch = -90.0f; 
    private float smyaw   =  0.0f;

    private float dsmtrans_x = 0.0f;
    private float dsmtrans_y = 0.0f;
    private float dsmtrans_z = 0.0f;
    private float dsmroll  = 0.0f;
    private float dsmpitch = 0.0f;
    private float dsmyaw   = 0.0f;

    private bool smlbutton = false;
    private bool smrbutton = false;

    // private float y_max =  0.28f;
    

    
    // "hover": 1,
    // "select": 2,
    // "translate": 3,
    // "rotate": 4,
    // "release": 5

    // public Teleop Copy(){
    //     return (Teleop) this.MemberwiseClone();
    // }

    int GetMinIndex(List<float> list)
    {
        if (list == null) {
            throw new ArgumentNullException("list");
        }

        if (list.Count == 0) {
            throw new ArgumentException("List is empty.", "list");
        }

        float min = list[0];
        int minIndex = 0;

        for (int i = 1; i < list.Count; ++i) {
            if (list[i] < min) {
                min = list[i];
                minIndex = i;
            }
        }
        return minIndex;
    }

    void PoseCallback(TeleopPoseMsg msg){
        trans_x = (float)msg.x;
        trans_y = (float)msg.y + yoffset;
        trans_z = (float)msg.z;
        quat_x  = (float)msg.x_;
        quat_y  = (float)msg.y_;
        quat_z  = (float)msg.z_;
        quat_w  = (float)msg.w_;
    }


    void SwPoseCallback(TeleopPoseMsg msg){
        swtrans_x = (float)msg.x;
        swtrans_y = (float)msg.y + yoffset;
        swtrans_z = (float)msg.z;
        swquat_x  = (float)msg.x_;
        swquat_y  = (float)msg.y_;
        swquat_z  = (float)msg.z_;
        swquat_w  = (float)msg.w_;
    }

    void SmPoseCallback(SpacemouseMsg msg){
        dsmtrans_x = (float)msg.xyz[0]   - dsmtrans_x;
        dsmtrans_y = ((float)msg.xyz[1]) - dsmtrans_y ;
        dsmtrans_z = ((float)msg.xyz[2]) - dsmtrans_z ;
        dsmroll    = ((float)msg.rpy[0] - dsmroll  ) * (float)90 * (float)(Math.PI);
        dsmpitch   = ((float)msg.rpy[1] - dsmpitch ) * (float)90 * (float)(Math.PI);
        dsmyaw     = ((float)msg.rpy[2] - dsmyaw   ) * (float)90 * (float)(Math.PI);
        smlbutton = (bool)msg.lbutton;
        smrbutton = (bool)msg.rbutton;

        dsmtrans_x = (float)msg.xyz[0];
        dsmtrans_y = ((float)msg.xyz[1]);
        dsmtrans_z = ((float)msg.xyz[2]);

        dsmroll  = (float)msg.rpy[0] * (float)90 * (float)(Math.PI); 
        dsmpitch = (float)msg.rpy[1] * (float)90 * (float)(Math.PI); 
        dsmyaw   = (float)msg.rpy[2] * (float)90 * (float)(Math.PI);  
    }

    void PrimitiveIdCallback(PrimitiveMsg msg){
        if (use_speech2text){
            primitive_id = msg.id;
        } 
    }

    void ApplyMaterial(GameObject obj, Color color){
        Material mat = obj.GetComponent<Renderer>().material;
        mat.color = color;
    }

    void InitObj(GameObject obj, Vector3 trans, Quaternion quat, bool Collide = false){
        obj.transform.SetPositionAndRotation(trans,quat);
        Collider obj_collider = obj.GetComponent<Collider>();
        if (Collide == false){
            obj_collider.enabled = !obj_collider.enabled;
        }
        else{
            obj_collider.enabled = true;
        }

    }

    void SpawnObj(GameObject Obj){
         if (use_opti){

            use_smartwatch = false;
            use_spacemouse = false;
            // Vector3 trans = new Vector3(trans_x,trans_y,trans_z);
            if (trans_y < 0.37f){
                trans_y = 0.37f;
            }
            Vector3 trans = new Vector3(trans_x,trans_y,trans_z);
            Quaternion quat  = new Quaternion(0.0f,0.0f,0.0f,1.0f);
            Obj.transform.SetPositionAndRotation(trans,quat);
        }
        else if(use_smartwatch){
            use_opti =false;
            use_spacemouse =false;
          
            Vector3    trans = new Vector3(swtrans_x,0.37f,0.6f);
            // Vector3    trans = new Vector3(swtrans_x,swtrans_y,swtrans_z);
            Quaternion quat  = new Quaternion(0.0f,0.0f,0.0f,1.0f);
            Obj.transform.SetPositionAndRotation(trans,quat);

        }
        
        else if(use_spacemouse){
            use_opti =false;
            use_smartwatch =false;    
            // "hover": 1,
            // "select": 2,
            // "translate": 3,
            // "rotate": 4,
            // "release": 5
            if (primitive_id == 1){
                HoverSelect(0.05f);
            }

            if ((primitive_id == 2) && (select_obj_id < Objects.Count)){

                Objects[select_obj_id].transform.SetParent(HandL.transform);
                primitive_id = 3;
                

            }
            
            if ((primitive_id == 5) && (select_obj_id < Objects.Count)){

               Objects[select_obj_id].transform.SetParent(null);
               primitive_id = 0;
            }

            if ((primitive_id == 3) || (primitive_id == 1) && (primitive_id != 4)){

                smtrans_x =ConditionalMotion(smtrans_x, dsmtrans_x, 0.8f);
                smtrans_y =ConditionalMotion(smtrans_y, dsmtrans_y, 0.8f);
                smtrans_z =ConditionalMotion(smtrans_z, dsmtrans_z, 0.8f);
                Vector3 trans = new Vector3(smtrans_x, smtrans_y, smtrans_z);
                Quaternion quat  = Quaternion.Euler(-smyaw,smpitch,-smroll);
                Obj.transform.SetPositionAndRotation(trans,quat);
            } 

            if ((primitive_id == 4) && (primitive_id != 3)){

                Vector3 trans = new Vector3(smtrans_x, smtrans_y, smtrans_z);
                smyaw   = ConditionalMotion(smyaw  , dsmyaw,    90);
                smpitch = ConditionalMotion(smpitch, dsmpitch,  90);
                smroll  = ConditionalMotion(smroll , dsmroll,   90);
                Quaternion quat  = Quaternion.Euler(-smyaw,smpitch,-smroll);
                Obj.transform.SetPositionAndRotation(trans,quat);
            }
            Debug.Log(string.Format("Primitive name: {0}",primitives[primitive_id]));
            ros.Publish(unityPrimitiveName,msg);
        }
    }
    

    float ConditionalMotion(float goal, float delta, float thresh = 0.1f){
        if (delta < thresh){
            goal += (delta * 0.1f);
        }
        else if (delta > thresh){
            goal -= (delta * 0.1f);
        }
        return goal;
    }

    void Select(){
        int tmp = select_obj_id;
        while (true){
            if ((primitive_id == 2) && (select_obj_id < Objects.Count)){
                Objects[select_obj_id].transform.SetParent(HandL.transform);
                if (select_obj_id != tmp){
                    Objects[tmp].transform.SetParent(null);
                }
                tmp = select_obj_id;
            }
        }
    }

    void HoverSelect(float thresh = 0.05f){
        Vector3 c_trans = HandL.transform.position;
        List<float> errors = new List<float> ();

        for (int i = 0; i < Objects.Count; i++){
            Vector3 object_trans  = Objects[i].transform.position;
            Vector3 error = new Vector3((c_trans.x - object_trans.x),
                                        (0.0f),
                                        // (c_trans.y - object_trans.y),
                                        (c_trans.z - 0.08f - object_trans.z));

            errors.Add(error.magnitude);
        }

        int min_idx = GetMinIndex(errors);
        float min_error = errors[min_idx];
        Debug.Log(string.Format("Min error mag: {0} for object {1}", min_error, min_idx+1));   

        if (min_error < thresh){
            hover_disp += 0.01f;
            if (hover_disp > hover_disp_max){
                hover_disp = hover_disp_max;
            }
            else if(hover_disp > c_trans.y){
                hover_disp = c_trans.y - 0.05f;
            }

            if (hover_disp < 0.35f){
                hover_disp = 0.35f;
            }
            select_obj_id = min_idx;
            Vector3 move = new Vector3(Objects[min_idx].transform.position.x, hover_disp, Objects[min_idx].transform.position.z);
            Objects[min_idx].transform.SetPositionAndRotation(move, Quaternion.Euler(0,0,0));
        }
        else{
            hover_disp -= 0.01f;
            if (hover_disp < 0.35f){
                hover_disp = 0.35f;
            }
            for (int idx = 0 ; idx < Objects.Count; idx++){
                if ((Objects[idx].transform.position.y > 0.35f) && (Objects[idx].transform.position.y <= hover_disp_max)){
                    Vector3 move = new Vector3(Objects[idx].transform.position.x, hover_disp, Objects[idx].transform.position.z);
                    Objects[idx].transform.SetPositionAndRotation(move, Quaternion.Euler(0,0,0));
                }
            }
            
        }
    }


    void SpaceMousePrimitiveChange(){

        while (true){
            // Debug.Log(string.Format("Left Button: {0} Right Button: {1}",smlbutton,smrbutton));
            if(smlbutton){
                primitive_id -= 1;
            }
            else if (smrbutton){
                primitive_id += 1;
            }

            if (primitive_id >5){
                primitive_id = prim_id_max;
            }
            else if(primitive_id<1){
                primitive_id = prim_id_min;
            }
            msg.data = primitives[primitive_id];
            Thread.Sleep(250);
        }
    }



    void Start(){
        Cube1       = GameObject.Find("Cube_1");
        Cube2       = GameObject.Find("Cube_2");
        Cube3       = GameObject.Find("Cube_3");
        Cube4       = GameObject.Find("Cube_4");
        Cube5       = GameObject.Find("Cube_5");
        Cube1_1     = GameObject.Find("EdgeCube_1");
        Cube2_1     = GameObject.Find("EdgeCube_2");
        Cube3_1     = GameObject.Find("EdgeCube_3");
        Cube4_1     = GameObject.Find("EdgeCube_4");
        Cube5_1     = GameObject.Find("EdgeCube_5");

        Cube1_shadow  = GameObject.Find("Cube_1_shadow");
        Cube2_shadow  = GameObject.Find("Cube_2_shadow");
        Cube3_shadow  = GameObject.Find("Cube_3_shadow");
        Cube4_shadow  = GameObject.Find("Cube_4_shadow");
        Cube5_shadow  = GameObject.Find("Cube_5_shadow");


        // Vector3 local_trans = new Vector3(0f,0.35f,0f);
        // Quaternion local_quat  = Quaternion.Euler(0f,0f,0f); 
        // Cube1_1.transform.SetLocalPositionAndRotation(local_trans,local_quat);
        // Cube2_1.transform.SetLocalPositionAndRotation(local_trans,local_quat);
        // Cube3_1.transform.SetLocalPositionAndRotation(local_trans,local_quat);
        // Cube4_1.transform.SetLocalPositionAndRotation(local_trans,local_quat);

        // Cube1_1.transform.SetParent(Cube1.transform);
        // Cube2_1.transform.SetParent(Cube2.transform);
        // Cube3_1.transform.SetParent(Cube3.transform);
        // Cube4_1.transform.SetParent(Cube4.transform);

        // Cube4       = GameObject.Find("Cube5");
        // GameObject cubencube = GameObject.Find("CubeNCube");
        // cubencube.transform.SetParent(Cube4.transform);
        // Cube4.transform.SetChild(GameObject.Find("CubeNCube"));
        HandL       = GameObject.Find("vr_hand_L");
        Objects = new List<GameObject> (){Cube1,Cube2,Cube3,Cube4,Cube5};
        Objects1 = new List<GameObject> (){Cube1_1,Cube2_1,Cube3_1,Cube4_1,Cube5_1};
        for (int i = 0; i < Objects.Count; ++i){
            InitObj(Objects[i], init_trans, init_quat, true);
            InitObj(Objects1[i], init_trans1, init_quat, true);
            Objects1[i].transform.SetParent(Objects[i].transform);
            ApplyMaterial(Objects1[i],Color.gray);
            init_trans[0] -= 0.2f;
            init_trans1[0] -= 0.2f;
            // init_trans[2] += 0.15f;
        }
       


        ApplyMaterial(Cube1,Color.red);
        ApplyMaterial(Cube2,Color.green);
        ApplyMaterial(Cube3,Color.cyan);
        ApplyMaterial(Cube4,Color.yellow);

        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TeleopPoseMsg>(teleopTopicName, PoseCallback);
        ros.Subscribe<TeleopPoseMsg>(swteleopTopicName, SwPoseCallback);
        ros.Subscribe<SpacemouseMsg>(smteleopTopicName, SmPoseCallback);
        ros.Subscribe<PrimitiveMsg>(primitveTopicName, PrimitiveIdCallback);
        ros.RegisterPublisher<StringMsg>(unityPrimitiveName);
        Thread t1   = new Thread(new ThreadStart(SpaceMousePrimitiveChange));
        // Thread t2  = new Thread(new ThreadStart(Select));
        t1.Start();
        // t2.Start();
        // Collider Cube_Collider = Cube.GetComponent<Collider>();
        // Cube_Collider.enabled = !Cube_Collider.enabled;
    }

    

  
    void Update(){      
        Debug.Log(string.Format("Primitive Id: {0}",primitive_id));
        SpawnObj(HandL);
        // SpawnObj(Objects[2]);
    }
}