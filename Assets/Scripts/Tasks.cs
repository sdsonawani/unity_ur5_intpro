// using System.Numerics;
// using System.Diagnostics;
using System.Threading;
// using System.Numerics;
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
// using System.IO;
// using System.Collections;
using System.Collections.Generic;
using Random=UnityEngine.Random;

public class Tasks: MonoBehaviour{

   
    public GameObject Cube1 ;
    public GameObject Cube2 ;
    public GameObject Cube3 ;
    public GameObject Cube4 ;
    public GameObject Cube5 ;

    public GameObject Cube1_1 ;
    public GameObject Cube2_1 ;
    public GameObject Cube3_1 ;
    public GameObject Cube4_1 ;
    public GameObject Cube5_1 ;
    public GameObject HandL ;
    private List<GameObject> Objects;
    private List<GameObject> Objects1;
    private List<GameObject> Objects2;
    public string primitveTopicName = "primitive_id";
    public float dx = 0.5f;
    private int primitive_id;

    private float waitTime = 2.0f;
    private List<Action> functions_;
    ROSConnection ros;

    private int func_id;
    void PrimitiveIdCallback(PrimitiveMsg msg){
        primitive_id = msg.id;
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

    void Start(){
        Cube1       = GameObject.Find("Cube_1_clone");
        Cube2       = GameObject.Find("Cube_2_clone");
        Cube3       = GameObject.Find("Cube_3_clone");
        Cube4       = GameObject.Find("Cube_4_clone");
        Cube5       = GameObject.Find("Cube_5_clone");
        Cube1_1     = GameObject.Find("EdgeCube_1_clone");
        Cube2_1     = GameObject.Find("EdgeCube_2_clone");
        Cube3_1     = GameObject.Find("EdgeCube_3_clone");
        Cube4_1     = GameObject.Find("EdgeCube_4_clone");
        Cube5_1     = GameObject.Find("EdgeCube_5_clone");
   

        Objects = new List<GameObject> (){Cube1,Cube2,Cube3,Cube4,Cube5};
        Objects1 = new List<GameObject> (){Cube1_1,Cube2_1,Cube3_1,Cube4_1,Cube5_1};

        for (int i = 0; i < Objects.Count; ++i){
            Objects1[i].transform.SetParent(Objects[i].transform);
            ApplyMaterial(Objects1[i],Color.gray);
            // ApplyMaterial(Objects2[i],Color.gray);
        }
        ApplyMaterial(Cube1,Color.red);
        ApplyMaterial(Cube2,Color.green);
        ApplyMaterial(Cube3,Color.cyan);
        ApplyMaterial(Cube4,Color.yellow);
        ApplyMaterial(Cube5,Color.blue);

        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PrimitiveMsg>(primitveTopicName, PrimitiveIdCallback);

        // Thread.Sleep(5000);
        // inverted_u();
        // O_shape();

        // functions_ = new List<Action>{ID1_shape,ID2_shape,ID3_shape,ID4_shape,ID5_shape,ID6_shape};
        functions_ = new List<Action>{ID1_shape,ID2_shape};
        func_id = (int)Random.Range(0,functions_.Count);
        // ID1_shape,ID2_shape,ID3_shape,ID4_shape,ID5_shape,ID6_shape}
        // StartCoroutine(waiter());

    }

    void reset_pose(){
        for(var i=0; i < Objects.Count; i++){
            Objects[i].transform.SetPositionAndRotation(new Vector3(0,0,0),Quaternion.Euler(0,0,0));
        }
    }
    void InvertedU_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.25f - dx,0.37f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.15f - dx,0.37f,0.50f),Quaternion.Euler(0,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.20f - dx,0.45f,0.50f),Quaternion.Euler(0,0,90));

    }

    void InvertedY_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.25f - dx,0.37f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.15f - dx,0.37f,0.50f),Quaternion.Euler(0,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.20f - dx,0.45f,0.50f),Quaternion.Euler(0,0,90));
        Cube4.transform.SetPositionAndRotation(new Vector3(0.20f - dx,0.525f,0.50f),Quaternion.Euler(0,0,0));

    }

    void O_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.225f- dx,0.40f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.175f- dx,0.40f,0.50f),Quaternion.Euler(0,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.20f - dx, 0.325f,0.50f),Quaternion.Euler(0,0,90));
        Cube4.transform.SetPositionAndRotation(new Vector3(0.20f - dx,0.475f,0.50f),Quaternion.Euler(0,0,90));

    }

    void T_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.25f,0.37f,0.50f),Quaternion.Euler(0,0,0));
        // Cube2.transform.SetPositionAndRotation(new Vector3(0.15f,0.37f,0.50f),Quaternion.Euler(0,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.25f,0.45f,0.50f),Quaternion.Euler(0,0,90));

    }

    void U_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.125f,0.35f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.275f,0.35f,0.50f),Quaternion.Euler(0,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.20f, 0.325f,0.50f),Quaternion.Euler(0,0,90));

    }

    void ID1_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.125f- dx,0.35f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.125f- dx,0.325f,0.575f),Quaternion.Euler(90,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.20f - dx, 0.325f,0.50f),Quaternion.Euler(0,0,90));
        Cube4.transform.SetPositionAndRotation(new Vector3(0.125f- dx, 0.325f,0.425f),Quaternion.Euler(-90,0,0));

    }

    void ID2_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.25f  - dx,  0.325f, 0.50f),Quaternion.Euler(0,0,90));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.17f  - dx, 0.325f,  0.425f),Quaternion.Euler(90,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.327f - dx, 0.325f, 0.425f),Quaternion.Euler(90,0,0));
        Cube4.transform.SetPositionAndRotation(new Vector3(0.327f - dx, 0.35f, 0.50f),Quaternion.Euler(0,0,0));
        Cube5.transform.SetPositionAndRotation(new Vector3(0.175f -  dx, 0.35f, 0.50f),Quaternion.Euler(0,0,0));
        // Cube4.transform.SetPositionAndRotation(new Vector3(0.045f, 0.325f,0.50f),Quaternion.Euler(0,0,0));
        // Cube3.transform.SetPositionAndRotation(new Vector3(0.045f, 0.325f,0.50f),Quaternion.Euler(0,0,90));

    }

    void ID3_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.125f - dx, 0.35f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.125f - dx, 0.35f,0.575f),Quaternion.Euler(-45, 0, 0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.045f - dx, 0.35f,0.50f),Quaternion.Euler(0, 0, -45));

    }


    void ID4_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.125f - dx, 0.35f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.125f - dx, 0.35f,0.575f),Quaternion.Euler(-45, 0, 0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.205f - dx, 0.35f,0.50f),Quaternion.Euler(0, 0, 45));

    }

    void ID5_shape(){

        Cube1.transform.SetPositionAndRotation(new Vector3(0.25f  - dx,0.35f,0.50f),Quaternion.Euler(0,0,0));
        Cube2.transform.SetPositionAndRotation(new Vector3(0.25f  - dx,0.425f,0.50f),Quaternion.Euler(0,0,90));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.275f - dx,0.50f,0.50f),Quaternion.Euler(0,0,0));
        Cube4.transform.SetPositionAndRotation(new Vector3(0.225f - dx,0.50f,0.50f),Quaternion.Euler(0,0,0));

    }

    void ID6_shape(){

        Cube2.transform.SetPositionAndRotation(new Vector3(0.25f - dx,0.325f,0.50f),Quaternion.Euler(0,0,90));
        Cube1.transform.SetPositionAndRotation(new Vector3(0.25f - dx,0.40f,0.50f),Quaternion.Euler(0,0,0));
        Cube3.transform.SetPositionAndRotation(new Vector3(0.25f - dx,0.47f,0.50f),Quaternion.Euler(0,0,90));
        // Cube4.transform.SetPositionAndRotation(new Vector3(0.215f,0.50f,0.50f),Quaternion.Euler(0,0,0));

    }


    // void U_shape(){

    //     Cube1.transform.SetPositionAndRotation(new Vector3(0.235f,0.45f,0.50f),Quaternion.Euler(0,0,0));
    //     Cube2.transform.SetPositionAndRotation(new Vector3(0.165f,0.45f,0.50f),Quaternion.Euler(0,0,0));
    //     Cube3.transform.SetPositionAndRotation(new Vector3(0.20f, 0.37f,0.50f),Quaternion.Euler(0,0,90));

    // }

    IEnumerator waiter(){
        
        while (true){
        

        ID1_shape();
        yield return new  WaitForSecondsRealtime(waitTime);
        reset_pose();

        ID2_shape();
        yield return new  WaitForSecondsRealtime(waitTime);
        reset_pose();

        // ID3_shape();
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();

        // ID4_shape();
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();

        // ID5_shape();
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();


        // ID6_shape();
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();


        // O_shape();
        // Debug.Log("Created O-shape");
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();

        // T_shape();
        // Debug.Log("Created T-shape");
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();
        

        // InvertedY_shape();
        // Debug.Log("Created inverted Y-shape");
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();
        
        // U_shape();
        // Debug.Log("Created U-shape");
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();

        // InvertedU_shape();
        // Debug.Log("Created inverted U-shape");
        // yield return new  WaitForSecondsRealtime(waitTime);
        // reset_pose();

        }
    }
    // "hover": 1,
    // "select": 2,
    // "translate": 3,
    // "rotate": 4,
    // "release": 5

    void Update(){
        functions_[func_id]();
        // Debug.Log(func_id);
        // O_shape();

        // T_shape();
        
        // InvertedY_shape();
        
        // U_shape();

        // InvertedU_shape();


    }
}