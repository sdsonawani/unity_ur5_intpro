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



public class CubeStacking: MonoBehaviour{

    public GameObject Cube_1;
    public GameObject CubeNCube;

    ROSConnection ros;
    public Shader shade;

    public float baseX = 0.0f;
    public float baseY = 0.35f;
    public float baseZ = 0.6f;


    void ApplyMaterial(GameObject obj, Color color){
        Material mat = obj.GetComponent<Renderer>().material;
        // mat.shader = shade;
        mat.color = color;

    }

    void ApplyProperties(GameObject obj, Vector3 trans, Quaternion quat, bool Collide = false){
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

        Cube_1 = GameObject.Find("Cube_4");
        CubeNCube = GameObject.Find("CubeNCube");

        Color Cube_1_Color      = new Color(0.0f, 0.9f, 0.0f, 1.0f);        
        Vector3 CubeTrans       = new Vector3(baseX, baseY, baseZ);
        ApplyProperties(Cube_1, CubeTrans, Quaternion.Euler(0,0,0), true);
        // ApplyMaterial(Cube_1, Cube_1_Color);


        Color CubeNCube_Color   = new Color(1f, 0f, 0f, 1.0f);
        Vector3 CubeNCubeTrans  = new Vector3(baseX, baseY + 0.05f, baseZ);
        ApplyProperties(CubeNCube, CubeNCubeTrans, Quaternion.Euler(0,0,0), true);
        // ApplyMaterial(CubeNCube, CubeNCube_Color);

    }

  

}