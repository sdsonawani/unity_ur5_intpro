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


public class Tasks: MonoBehaviour{

    public GameObject Cube1 ;
    public GameObject Cube2 ;
    public GameObject Cube3 ;
    public GameObject Cube4 ;
    public string primitveTopicName = "primitive_id";

    private int primitive_id;
    private List<GameObject> Objects;
    ROSConnection ros;

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
        Cube1       = GameObject.Find("Cube_1");
        Cube2       = GameObject.Find("Cube_2");
        Cube3       = GameObject.Find("Cube_3");
        Cube4       = GameObject.Find("Cube_4");
        Objects = new List<GameObject> (){Cube1,Cube2,Cube3,Cube4};
  
        ApplyMaterial(Cube1,Color.red);
        ApplyMaterial(Cube2,Color.green);
        ApplyMaterial(Cube3,Color.cyan);
        ApplyMaterial(Cube4,Color.yellow);

        ros =  ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PrimitiveMsg>(primitveTopicName, PrimitiveIdCallback);
    }


    // "hover": 1,
    // "select": 2,
    // "translate": 3,
    // "rotate": 4,
    // "release": 5

    void Update(){
    }
}