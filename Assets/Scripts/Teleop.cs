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
    private List<GameObject> Objects2;

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
    private Vector3    init_trans  = new Vector3(0.3f, 0.37f, 0.35f);
    private Vector3    init_trans1 = new Vector3(0.3f, 0.32f, 0.35f);
    private Vector3    init_trans2 = new Vector3(0.3f, 0.30f, 0.35f);
    private Quaternion init_quat  = new Quaternion(0,0,0,1);

    public Vector3    FrozenTrans = new Vector3(0.45f, 0.35f, 0.30f);
    public Quaternion FrozenQuat  = Quaternion.Euler(0,-90,0);

    public int obj_id  = 1;
    public int prim_id_max = 5;
    public int prim_id_min = 1;
    public int primitive_id;
    public int select_obj_id = 10;
    public int tmp = 100;

    public float hover_disp = 0.35f;
    public float hover_disp_max = 0.40f;

    private List<string> primitives = new List<string> (){"init","hover", "select", "translate", "rotate", "release"}; 
    private StringMsg msg = new StringMsg();
     
    private float trans_x, trans_y, trans_z;
    private float dtrans_x, dtrans_y, dtrans_z;
    private float opti_trans_x = 0;
    private float opti_trans_y = 0;
    private float opti_trans_z = 0;
    private float tx = 0f;
    private float ty = 0.35f;
    private float tz = 0.35f;
    private float roll = 0.0f;
    private float pitch = -90f;
    private float yaw = 0f;

    private float opti_roll, opti_pitch, opti_yaw;
    private float droll, dpitch, dyaw;
    private float _roll =0;
    private float _pitch = -90;
    private float _yaw =0 ;


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
    private float delta_steps = 0.01f;
    private float original_size = 0.05f;
    public float delta_adjust = 0.01f;
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
    
        Vector3 e_ = new Quaternion(quat_x,quat_y,quat_z, quat_w).eulerAngles;

        droll  = e_[0] - opti_roll;
        dpitch = e_[1] - opti_pitch;
        dyaw   = e_[2] - opti_yaw;

        dtrans_x = trans_x - opti_trans_x;
        dtrans_y = trans_y - opti_trans_y;
        dtrans_z = trans_z - opti_trans_z;

        opti_trans_x = trans_x;
        opti_trans_y = trans_y;
        opti_trans_z = trans_z;

        opti_roll  = e_[0];  
        opti_pitch = e_[1]; 
        opti_yaw   = e_[2];

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
      
            if (primitive_id == 1){
                if (trans_y < 0.37f){
                    trans_y = 0.37f;
                }
                // Vector3 trans = new Vector3(trans_x,trans_y,trans_z);
                // // Quaternion quat  = new Quaternion(0.0f,0.0f,0.0f,1.0f);
                // Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
                // Obj.transform.SetPositionAndRotation(trans,quat);
                HoverSelect(0.05f);
            }

            // if ((primitive_id == 2) && (select_obj_id < Objects.Count)){
            if ((primitive_id == 2)){
                primitive_id = 3;
                if (select_obj_id < Objects.Count){
                    Objects[select_obj_id].transform.SetParent(HandL.transform);
                    Destroy(Objects[select_obj_id].GetComponent<Rigidbody>());
                    Destroy(Objects[select_obj_id].GetComponent<Collider>());
                }
            }
            
            if ((primitive_id == 3) || (primitive_id == 1) && (primitive_id != 4)){
                
                // opti_trans_x = dtrans_x;
                // opti_trans_y = dtrans_y;
                // opti_trans_z = dtrans_z;
                // tx = ConditionalMotion(tx,dtrans_x,0.01f);
                // ty = ConditionalMotion(ty,dtrans_y,0.01f);
                // tz = ConditionalMotion(tz,dtrans_z,0.01f);
                
                // Debug.Log(string.Format("trans_x: {0}",dtrans_x));
                var thresh  = 0.05f;
                if (Mathf.Abs(dtrans_x) < thresh && Mathf.Abs(dtrans_y) < thresh && Mathf.Abs(dtrans_z) < thresh){
                    tx += (3.0f *dtrans_x);
                    ty += (3.0f *dtrans_y);
                    tz += (3.0f *dtrans_z); 
                }
               
                Vector3 trans = new Vector3(tx, ty, tz);
                // Vector3 trans = new Vector3(trans_x, trans_y, dtrans_z);
                // Vector3 trans = new Vector3(dtrans_x, dtrans_y, dtrans_z);
                // Vector3 trans = new Vector3(opti_trans_x, opti_trans_y, opti_trans_z);
                // Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
                // Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
                Quaternion quat  = Quaternion.Euler(_roll,_pitch,_yaw);
                Obj.transform.SetPositionAndRotation(trans,quat);
            } 

            if ((primitive_id == 4) && (primitive_id != 3)){
                

                var thresh  = 1.0f;
                Debug.Log(string.Format("droll: {0}, dpitch: {1}, dyaw: {2}",droll, dpitch, dyaw));
                if (Mathf.Abs(droll) < thresh && Mathf.Abs(dpitch) < thresh && Mathf.Abs(dyaw) < thresh){
                _roll  += (1.50f * droll);
                _pitch += (1.50f * dpitch);
                _yaw   += (1.50f * dyaw);}

                Debug.Log(string.Format("roll delta: {0}",droll));
                Vector3 trans = new Vector3(tx, ty, tz);
                // Vector3 trans = new Vector3(opti_trans_x, opti_trans_y, opti_trans_z);
                // Vector3 trans = new Vector3(trans_x, trans_y, trans_z);
                // Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
                // Vector3 trans = new Vector3(opti_trans_x, opti_trans_y, opti_trans_z);
                // Quaternion quat  = new Quaternion(quat_x,quat_y,quat_z,quat_w);
                Quaternion quat  = Quaternion.Euler(_roll,_pitch,_yaw);
                Vector3 euler = quat.eulerAngles;
                roll = euler[0];
                pitch = euler[1];
                yaw = euler[2];
                Obj.transform.SetPositionAndRotation(trans,quat);
            }
            
            if ((primitive_id == 5) ){
            
            if (select_obj_id < Objects.Count){
               Objects[select_obj_id].transform.SetParent(null);
               Objects[select_obj_id].AddComponent<Rigidbody>();
            //    Objects[select_obj_id].AddComponent<Collider>();
               select_obj_id =  100;
            }
               primitive_id = 0;
               roll = 0;
               pitch = -90;
               yaw = 0;
               Debug.Log("reseted the angular values");
            }

            // Debug.Log(string.Format("Primitive name: {0}",primitives[primitive_id]));
            ros.Publish(unityPrimitiveName,msg);
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
                Destroy(Objects[select_obj_id].GetComponent<Rigidbody>());
                Destroy(Objects[select_obj_id].GetComponent<Collider>());
                

            }
            
            if ((primitive_id == 5) && (select_obj_id < Objects.Count)){

               Objects[select_obj_id].transform.SetParent(null);
               Objects[select_obj_id].AddComponent<Rigidbody>();
               Rigidbody body = Objects[select_obj_id].GetComponent<Rigidbody>();
               body.mass = 0.0f;
               select_obj_id = 100;
            //    Objects[select_obj_id].AddComponent<Collider>();

               smroll = 0;
               smpitch = -90;
               smyaw = 0;
               primitive_id = 0;
            }

            if ((primitive_id == 3) || (primitive_id == 1) && (primitive_id != 4)){

                smtrans_x =ConditionalMotion(smtrans_x, dsmtrans_x, 0.8f);
                smtrans_y =ConditionalMotion(smtrans_y, dsmtrans_y, 0.8f);
                smtrans_z =ConditionalMotion(smtrans_z, dsmtrans_z, 0.8f);
                if(smtrans_y < 0.35f){
                    smtrans_y = 0.35f;
                }
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
            // Debug.Log(string.Format("Primitive name: {0}",primitives[primitive_id]));
            ros.Publish(unityPrimitiveName,msg);
        }

    // Moving_and_Scaling_Shadow();
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

    void HoverSelect(float thresh = 0.01f){
        Vector3 c_trans = HandL.transform.position;
        List<float> errors = new List<float> ();

        for (int i = 0; i < Objects.Count; ++i){
            Vector3 object_trans  = Objects[i].transform.position;
            Vector3 error = new Vector3((c_trans.x - object_trans.x),
                                        (0.0f),
                                        // (c_trans.y - object_trans.y),
                                        (c_trans.z - 0.08f - object_trans.z));
                                        // (c_trans.z - 0.00f - object_trans.z));

            errors.Add(error.magnitude);
        }

        int min_idx = GetMinIndex(errors);
        float min_error = errors[min_idx];
        // Debug.Log(string.Format("Min error mag: {0} for object {1}", min_error, min_idx+1));   

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
            // Objects[min_idx].transform.SetPositionAndRotation(move, Quaternion.Euler(0,0,0));
            }
        else {
            hover_disp -= 0.01f;
            if (hover_disp < 0.35f){
                hover_disp = 0.35f;
            }
            for (int idx = 0 ; idx < Objects.Count; ++idx){
                if ((Objects[idx].transform.position.y > 0.35f) && (Objects[idx].transform.position.y <= hover_disp_max)){
                    Vector3 move = new Vector3(Objects[idx].transform.position.x, 0.35f, Objects[idx].transform.position.z);
                    // Objects[idx].transform.SetPositionAndRotation(move, Quaternion.Euler(0,0,0));
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

    void Moving_and_Scaling_Shadow(){
        for (int i = 0 ; i < Objects2.Count; i++){
                var parent_y = Objects[i].transform.position[1];
                var child_y = Objects2[i].transform.position[1];
                var delta = (parent_y - child_y);
                Debug.Log(string.Format("Delta distance in meter: {0}",delta));
                if (delta>0){
                    Objects2[i].transform.localScale = new Vector3((float) (original_size - delta_adjust * delta), 0.001f, (float) (original_size - delta_adjust * delta));
                    Objects2[i].transform.SetPositionAndRotation(new Vector3(Objects[i].transform.position[0], 0.325f, Objects[i].transform.position[2]),
                                                                    Objects[i].transform.rotation);
                }
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
        Objects = new List<GameObject>  (){Cube1,Cube2,Cube3,Cube4,Cube5};
        Objects1 = new List<GameObject> (){Cube1_1,Cube2_1,Cube3_1,Cube4_1,Cube5_1};
        Objects2 = new List<GameObject> (){Cube1_shadow,Cube2_shadow,Cube3_shadow,Cube4_shadow,Cube5_shadow};
        for (int i = 0; i < Objects.Count; ++i){
            InitObj(Objects[i], init_trans, init_quat, true);
            InitObj(Objects1[i], init_trans1, init_quat, true);
            // InitObj(Objects2[i], init_trans1, init_quat, false);
            Objects1[i].transform.SetParent(Objects[i].transform);
            // Objects2[i].transform.SetParent(Objects[i].transform);
            ApplyMaterial(Objects1[i],Color.gray);
            // ApplyMaterial(Objects2[i],Color.gray);
            init_trans [2] += 0.1f;
            init_trans1[2] += 0.1f;
            init_trans2[2] += 0.1f;
            // init_trans[2] += 0.15f;
        }

        ApplyMaterial(Cube1,Color.red);
        ApplyMaterial(Cube2,Color.green);
        ApplyMaterial(Cube3,Color.cyan);
        ApplyMaterial(Cube4,Color.yellow);
        ApplyMaterial(Cube5,Color.blue);

        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TeleopPoseMsg>(teleopTopicName, PoseCallback);
        ros.Subscribe<TeleopPoseMsg>(swteleopTopicName, SwPoseCallback);
        ros.Subscribe<SpacemouseMsg>(smteleopTopicName, SmPoseCallback);
        ros.Subscribe<PrimitiveMsg>(primitveTopicName, PrimitiveIdCallback);
        ros.RegisterPublisher<StringMsg>(unityPrimitiveName);
        Thread t1   = new Thread(new ThreadStart(SpaceMousePrimitiveChange));
        // Thread t2   = new Thread(new ThreadStart(Moving_and_Scaling_Shadow));
        // Thread t2  = new Thread(new ThreadStart(Select));
        t1.Start();
        // t2.Start();
        // Collider Cube_Collider = Cube.GetComponent<Collider>();
        // Cube_Collider.enabled = !Cube_Collider.enabled;
    }

    

  
    void Update(){      
        // Debug.Log(string.Format("Primitive Id: {0}",primitive_id));
        SpawnObj(HandL);
        // SpawnObj(Objects[0]);
    }
}