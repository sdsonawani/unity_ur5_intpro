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


public class HumanFocus : MonoBehaviour
{
    public GameObject Cube;
    public GameObject FrontCube;
    public GameObject BackCube;
    public GameObject UR5;
    public Camera POV = new();

    private ROSConnection ros;
    public string topicName = "pov_pose";
    public string camTopicName = "intention_image";
    public string cubeTopicName = "cube_pose";
    public float trans_x, trans_y, trans_z;
    public float quat_w, quat_x, quat_y, quat_z;
    public float offset = 0;
    public HeaderMsg msg_ = new();

    private float x_min = -0.67f;
    private float x_max = 0.59f;
    private float z_min = 0.05f;
    private float z_max = 0.80f;
    private float y_min = 0.29f;
    private float y_max = 0.28f;
    private Vector3 Normal;
    private float a;
    private float b;
    private float c;
    private float d;

    private Camera cam;

    private void Start()
    {
        Cube = GameObject.Find("Cube");
        FrontCube = GameObject.Find("front_cube");
        BackCube = GameObject.Find("back_cube");


        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CamPoseMsg>(topicName, PoseCallback);
        ros.RegisterPublisher<CamPoseMsg>(cubeTopicName);
        var cube_trans = new Vector3(0.0f, 1.8f, 0.384f);
        var cube_quat = Quaternion.Euler(0, 0, 0);
        Cube.transform.SetPositionAndRotation(cube_trans, cube_quat);

        FrontCube.transform.SetParent(POV.transform);
        FrontCube.transform.localPosition = new Vector3(0.0f, 0.0f, 0.0f);
        var FCube_Collider = FrontCube.GetComponent<Collider>();
        FCube_Collider.enabled = !FCube_Collider.enabled;
        BackCube.transform.SetParent(POV.transform);
        BackCube.transform.localPosition = new Vector3(0.0f, 0.0f, -0.1f);
        var BCube_Collider = BackCube.GetComponent<Collider>();
        BCube_Collider.enabled = !BCube_Collider.enabled;

        var Cube_Collider = Cube.GetComponent<Collider>();
        Cube_Collider.enabled = !Cube_Collider.enabled;
        GetPlane();
    }


    private void GetPlane()
    {
        // ref: https://keisan.casio.com/exec/system/1223596129
        var A = new Vector3(x_min, y_max, z_min);
        var B = new Vector3(-0.2f, y_max, z_max);
        var C = new Vector3(0.2f, y_max, 0.5f);
        var AB = A - B;
        var AC = A - C;
        Normal = Vector3.Cross(AB, AC).normalized;
        a = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
        b = (B.z - A.z) * (C.z - A.z) - (C.z - A.z) * (B.x - A.x);
        c = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
        d = -(a * A.x + b * A.y + c * A.z);
        // Debug.Log(string.Format("a: {0}, b: {1}, c: {2}, s: {3}", a,b,c,d));
        // Debug.Log(string.Format("Vector AB: {0}", AB));
        // Debug.Log(string.Format("Vector AC: {0}", AC));
    }

    private Vector3 GetIntersection(Vector3 lx, Vector3 ly)
    {
        // GetPlane();
        var t_ = (-d - a * lx.x - b * lx.y - c * lx.z) / (a * ly.x + b * ly.y + c * ly.z);
        var _x = lx.x + t_ * ly.x;
        var _y = lx.y + t_ * ly.y;
        var _z = lx.z + t_ * ly.z;
        // Debug.Log(string.Format("x: {0}, y: {1}, z: {2}", _x, _y, _z));
        var Intersection = new Vector3(_x, _y, _z);
        return Intersection;
    }

    private void PoseCallback(CamPoseMsg msg)
    {
        trans_x = (float)msg.x;
        trans_y = (float)msg.y;
        trans_z = (float)msg.z;
        quat_x = (float)msg.x_;
        quat_y = (float)msg.y_;
        quat_z = (float)msg.z_;
        quat_w = (float)msg.w_;
    }

    public void BgrToRgb(byte[] data)
    {
        for (var i = 0; i < data.Length; i += 3)
        {
            var dummy = data[i];
            data[i] = data[i + 2];
            data[i + 2] = dummy;
        }
    }

    private void Update()
    {
        var FC_GTrans = new Vector3();
        var BC_GTrans = new Vector3();
        var FC_GQuat = new Quaternion();
        var BC_GQuat = new Quaternion();

        FC_GTrans = FrontCube.transform.position;
        BC_GTrans = BackCube.transform.position;

        var LineVector = FC_GTrans - BC_GTrans;
        var Intersection = GetIntersection(FC_GTrans, LineVector);

        // Debug.Log(string.Format("Front_cube position ${0}",FC_GTrans));
        Debug.Log(string.Format("Intersection position ${0}", Intersection));


        // Vector3 trans = new Vector3(trans_x,trans_y+0.3f,trans_z+0.2f);
        var trans = new Vector3(trans_x, trans_y, trans_z);
        var quat = new Quaternion(quat_x, quat_y, quat_z, quat_w);
        POV.transform.SetPositionAndRotation(trans, quat);
        Cube.transform.SetPositionAndRotation(Intersection, FC_GQuat);

        var cubePoseMsg = new CamPoseMsg();
        // cubePoseMsg.x = Intersection.x;
        // cubePoseMsg.y = Intersection.y;
        // cubePoseMsg.z = Intersection.z;
        cubePoseMsg.x = trans_x;
        cubePoseMsg.y = trans_y;
        cubePoseMsg.z = trans_z;
        ros.Publish(cubeTopicName, cubePoseMsg);

        // POV.transform.LookAt(UR5.transform); 
    }
}