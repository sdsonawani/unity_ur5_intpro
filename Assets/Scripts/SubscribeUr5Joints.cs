using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Ur5Intpro;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using System;
using System.Threading;
using System.Collections;

// ref links
// 1) https://github.com/Unity-Technologies/URDF-Importer/blob/main/com.unity.robotics.urdf-importer/Runtime/Controller/JointControl.cs
// 2) https://github.com/Unity-Technologies/URDF-Importer/blob/main/com.unity.robotics.urdf-importer/Runtime/Controller/Controller.cs


public enum ControlType { PositionControl };
// public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
public class SubscribeUr5Joints : MonoBehaviour
{
    public GameObject ur5;
    public GameObject ur5_delta;
    public float publisheFreq = 30f;
    public float deltaTime = 0.1f;
    private float publishMessageTime;
    private float timeElapsed1;
    private float timeElapsed2;
    private float waitTimeElapsed;
    public int num_joints = 6;
    public double[] _joints = new double[6];
    public static readonly string[] LinkNames ={"world/dummy_link/base_link/shoulder_link",
                                                "/upper_arm_link", 
                                                "/forearm_link", 
                                                "/wrist_1_link",  
                                                "/wrist_2_link",  
                                                "/wrist_3_link" };
    ArticulationBody[] m_ur5ArticulationBody;
    ArticulationBody[] m_ur5ArticulationBody_delta;



    public float p_gain = 5.0f;
    public float i_gain = 0.8f;
    public float d_gain = 1.0f;
    public float[] p_istate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    public float[] p_dstate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    public float m_i_max = (float)Math.PI;
    public float m_i_min = -(float)Math.PI;

    private float curr_j = 0.0f;
    

    void Start()
    {   
        publishMessageTime = (1/publisheFreq);
        // find specific game object
        ur5 = GameObject.Find("ur5");
        ur5_delta = GameObject.Find("ur5_1");
        // ros subscriber 
        // ROSConnection.GetOrCreateInstance().Subscribe<Ur5JointsMsg>("real_ur5/joints", jointCallback);
        ROSConnection.GetOrCreateInstance().Subscribe<Ur5JointsMsg>("ur5_goal/joints", jointCallback);
        
        // load articulate body with non static joints
        var link_names              = string.Empty;
        m_ur5ArticulationBody       = new ArticulationBody[num_joints];
        m_ur5ArticulationBody_delta = new ArticulationBody[num_joints];
        for (var i = 0; i < num_joints; i++){
            link_names += LinkNames[i];
            m_ur5ArticulationBody[i]       = ur5.transform.Find(link_names).GetComponent<ArticulationBody>();
            m_ur5ArticulationBody_delta[i] = ur5_delta.transform.Find(link_names).GetComponent<ArticulationBody>();
            // Debug.Log(string.Format("Articulate body: {0} and joint position: {1}",m_ur5ArticulationBody[i],m_ur5ArticulationBody[i].jointPosition[0]));
        }


        // StartCoroutine(waiterTest());
    }
    
    // callback for joint angles published over specified ros topic
    void jointCallback(Ur5JointsMsg JointMsg)
    {
        for (var i = 0 ; i<num_joints; i++){
            _joints[i] = JointMsg.ur_joints_unity[i];
        }
    }

    void applyControl(ArticulationBody[] ur5_body){
        // init debug values for current and goal joints
            float[] joints_c_debug = new float[6]; 
            float[] joints_g_debug = new float[6]; 
          
            // apply control to each joint [0...5]
            for (var i = 0 ; i < num_joints; i++){

                var currentJDrive = ur5_body[i].xDrive;

                // real ur5 and sim ur5 urdf offset for current joint angle (radian)
                if (i == 1){
                    curr_j = (float)ur5_body[i].jointPosition[0] - (float)(Math.PI/2.0);
                }
                else if( i ==3){
                
                    curr_j = (float)ur5_body[i].jointPosition[0] - (float)(Math.PI/2.0);
                }
                else{
                    curr_j = (float)ur5_body[i].jointPosition[0];
                }                

                // get goal joint angle (radian)
                var goal_j = _joints[i];

                // error 
                var error = goal_j - curr_j;

                // apply pid controller
                // avoid integral winding  
                p_istate[i] += (float)error;
                if (p_istate[i] > m_i_max){
                    p_istate[i] = m_i_max;
                }
                else if (p_istate[i] < m_i_min){
                    p_istate[i] = m_i_min;
                }
                var velocity  = (p_gain * error) + (p_istate[i] * i_gain) + ((error - p_dstate[i]) * d_gain);
                p_dstate[i] = (float)error;

                // apply target position and velocity
                currentJDrive.damping = 1e+18f;
                currentJDrive.stiffness = 1e+18f;
                currentJDrive.forceLimit = 1e+20f;
                currentJDrive.target = (float)(goal_j * 180 / Math.PI);
                currentJDrive.targetVelocity = (float)(velocity * 180 / Math.PI);
                ur5_body[i].xDrive = currentJDrive;
                
                // load debug values
                joints_c_debug[i] = (float)ur5_body[i].xDrive.target;
                joints_g_debug[i] = (float)(goal_j * 180 / Math.PI);
            
            }

            // show debug values
            // Debug.Log(string.Format("current joints: [{0},{1},{2},{3},{4},{5}]",joints_c_debug[0], joints_c_debug[1], joints_c_debug[2], joints_c_debug[3], joints_c_debug[4], joints_c_debug[5]));            
            // Debug.Log(string.Format("goal joints: [{0},{1},{2},{3},{4},{5}]",joints_g_debug[0], joints_g_debug[1], joints_g_debug[2], joints_g_debug[3], joints_g_debug[4], joints_g_debug[5]));            
    }
    

    IEnumerator waiterTest(){

        timeElapsed1 += Time.deltaTime;
        if (timeElapsed1 > publishMessageTime){ 
            applyControl(m_ur5ArticulationBody);
            timeElapsed1 = 0;
        }

        yield return new WaitForSeconds(deltaTime);
        

        timeElapsed2 += Time.deltaTime;
        if (timeElapsed2 > publishMessageTime){ 
            applyControl(m_ur5ArticulationBody_delta);
            timeElapsed2 = 0;
        }

    }


    void Update(){
        
        timeElapsed1 += Time.deltaTime;
        publishMessageTime = (1/publisheFreq);
        if (timeElapsed1 > publishMessageTime)
        {   
            
            applyControl(m_ur5ArticulationBody);
            applyControl(m_ur5ArticulationBody_delta);

            timeElapsed1 = 0;
            
        }
    }
}