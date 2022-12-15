using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using System;

// ref links
// 1) https://github.com/Unity-Technologies/URDF-Importer/blob/main/com.unity.robotics.urdf-importer/Runtime/Controller/JointControl.cs
// 2) https://github.com/Unity-Technologies/URDF-Importer/blob/main/com.unity.robotics.urdf-importer/Runtime/Controller/Controller.cs


public enum ControlType { PositionControl };
// public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
public class SubscribeUr5Joints : MonoBehaviour
{
    public GameObject ur5;
    public float publishMessageFrequency = 0.01f;
    private float timeElapsed;
    public int num_joints = 6;
    public double[] joints_ = new double[6];
    public static readonly string[] LinkNames ={"world/dummy_link/base_link/shoulder_link",
                                                "/upper_arm_link", 
                                                "/forearm_link", 
                                                "/wrist_1_link",  
                                                "/wrist_2_link",  
                                                "/wrist_3_link" };
    UrdfJointRevolute[] m_jab;

    private  ArticulationBody[] articulationChain;
    // [InspectorReadOnly(hideInEditMode: true)]
    // public string selectedJoint;
    // [HideInInspector]
    // public int selectedIndex;


    // public ControlType control = ControlType.PositionControl;
    public float stiffness = 1000000f;
    public float damping = 100000f;
    public float forceLimit = 100000f;
    public float speed = 90f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 90f;// Units: m/s^2 / degree/s^2
    


    void Start()
    {
        // Controller setup
        articulationChain = ur5.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;

        foreach (ArticulationBody joint in articulationChain)
            {
                joint.gameObject.AddComponent<JointControl>();
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = forceLimit;
                joint.xDrive = currentDrive;
            }

        ROSConnection.GetOrCreateInstance().Subscribe<Ur5JointsMsg>("real_ur5/joints", jointCallback);
        var link_names = string.Empty;
        m_jab = new UrdfJointRevolute[num_joints];
        for (var i = 0; i < num_joints; i++){
            link_names += LinkNames[i];
            m_jab[i] = ur5.transform.Find(link_names).GetComponent<UrdfJointRevolute>();
        }
    }

    void jointCallback(Ur5JointsMsg JointMsg)
    {
        for (var i = 0 ; i<num_joints; i++){
            joints_[i] = JointMsg.ur_joints_unity[i];
        }
    }

    void Update(){
        var p_gain = 1.45;
        var thresh = 0.01;
        var curr_j = 0.0;

        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {      
            for (var i = 0 ; i<num_joints; i++){
                JointControl current = articulationChain[3+i].GetComponent<JointControl>();
                if (i == 1 || i == 3){
                    curr_j = m_jab[i].GetPosition() - (Math.PI/2.0);
                }
                else{
                    curr_j = m_jab[i].GetPosition();
                }

                var goal_j = joints_[i];
                var error = goal_j - curr_j;
                var c_value  = p_gain * error;
                if (c_value > thresh){
                    current.direction = RotationDirection.Positive ;
                }
                else if(c_value <= thresh ){
                    current.direction = RotationDirection.Negative ;
                }
                else{
                    current.direction = RotationDirection.None ;
                }
                // Debug.Log(string.Format("{0}, {1}",curr_j, goal_j));
                // Debug.Log(string.Format("{0}",joints_[i]));
                // i = 1;
                // Debug.Log(string.Format("{0}",m_jab[i]));
                // m_jab[i].SetAngle(joints_[i]);
            }

            timeElapsed = 0;
        }
    }
}