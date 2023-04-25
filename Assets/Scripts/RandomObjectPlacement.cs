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
using Random=UnityEngine.Random;
using Dummiesman;

public class RandomObjectPlacement: MonoBehaviour{

    public int x_grid = 5;
    public int z_grid = 5;
    private int count = 0;

    public List<string> object_names;
    public List<GameObject> objects;


    private Vector3 init_pose_ref = new Vector3(-0.70f, 1.95f, 0.4f); 
    private Vector3 init_pose1 = new Vector3(-0.70f, 1.95f, 0.4f); 
    private Vector3 local_scale = new Vector3(0.05f,0.05f,0.05f); 
    private string resource_dir = string.Format("/home/slocal/Documents/ur5_intpro/Assets/Resources/");
    
    private SerializedObject tagManager;
    private SerializedProperty tagsProp;

    private int _counter = 0;
 
    GameObject load_object(string object_name, int layer_id){
        GameObject gameobject = new OBJLoader().Load(string.Format("Assets/Resources/{0}.obj",object_name)); 
        
        GameObject child_object = Resources.Load<GameObject>(object_name).transform.GetChild(0).gameObject;
        child_object.transform.localScale = local_scale;
        gameobject.transform.localScale = local_scale;

        Rigidbody body = gameobject.AddComponent<Rigidbody>();
        body.angularDrag = 0.0f;
        // body.constraints = RigidbodyConstraints.FreezeRotationY;
        // body.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.constraints = RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        // body.constraints = RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
        body.useGravity = false;

        MeshCollider mesh_ = gameobject.AddComponent<MeshCollider>();
        var mesh = child_object.GetComponent<MeshFilter>().sharedMesh;
        mesh_.sharedMesh = mesh;
        mesh_.convex = true;
        gameobject.transform.GetChild(0).gameObject.transform.localRotation = Quaternion.Euler(180, 180, 0);

        // change the material properties
        var Renderer =  gameobject.transform.GetChild(0).gameObject.GetComponent<Renderer>();
        Renderer.material.SetColor("_SpecColor",Color.black);
        
        // add render layer id to object
        // gameobject.transform.GetChild(0).gameObject.layer = LayerMask.NameToLayer(string.Format("render_layer_{0}",layer_id));
        // gameobject.layer = LayerMask.NameToLayer(string.Format("render_layer_{0}",layer_id));
        // gameobject.tag = tag_id;
        return gameobject;
    }

    List<string> get_object_names(string resource_dir){

        List<string> local_list = new List<string>();
        var flag = Directory.Exists(resource_dir);
        Debug.Log(string.Format("checking if directory exist: {0}",flag));
        string[] files = Directory.GetFiles(resource_dir);
        for (var i = 0; i < files.Length; i++){
            string[] split_string = files[i].Split('.');
            var file_format = split_string[^1];
            if (file_format == "obj"){
                var object_name = files[i].Split('/')[^1].Split('.')[0];
                local_list.Add(object_name);
            }
        }
        return local_list;
    }

    void Start(){

        // tagManager = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
        // tagsProp   =  tagManager.FindProperty("tags");

        int layer_counter = 4;
        object_names = get_object_names(resource_dir);
        Debug.Log(string.Format("total objects found in resources: {0}",object_names.Count));

        for (var i = 0; i < x_grid; i++){
            for (var j = 0; j < z_grid; j++){
                // tagsProp.InsertArrayElementAtIndex(0);
                // SerializedProperty n = tagsProp.GetArrayElementAtIndex(0);
                // n.stringValue = string.Format("{0}",tag_counter);
                // tagManager.ApplyModifiedProperties();
                // Debug.Log(string.Format("{0}", tag_counter));

                var object_name = object_names[Random.Range(i+j+1,object_names.Count)];
                var game_object = load_object(object_name, layer_counter);

                game_object.transform.SetPositionAndRotation(init_pose1, new Quaternion(0,0,0,1));
                objects.Add(game_object);

                init_pose1[2] += 0.15f;
                layer_counter += 1;
            }
            init_pose1[2] = init_pose_ref[2];
            init_pose1[0] += 0.15f;
        }
    }

    void Update() {
        if (_counter == 100){
            
            for (var i = 0; i < objects.Count; i++){
            Rigidbody body = objects[i].GetComponent<Rigidbody>();
            body.useGravity = true;
            }
        }

        if (_counter == 150){
            Debug.Log("Applying constraints.......");
            for (var i = 0; i < objects.Count; i++){
            Rigidbody body = objects[i].GetComponent<Rigidbody>();
            body.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezePositionZ |  RigidbodyConstraints.FreezePositionX;
            body.velocity = Vector3.zero;

            }
        }


        _counter += 1;
    }

}
