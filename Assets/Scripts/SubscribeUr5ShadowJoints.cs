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


public class SubscribeUr5ShadowJoints : MonoBehaviour
{
    public GameObject ur5;
    public float publisheFreq = 30f;
    public float deltaTime = 0.1f;
    private float publishMessageTime;
    private float timeElapsed1;
    private float timeElapsed2;
    private float waitTimeElapsed;
    public int num_joints = 6;
    public int ee_joints = 3;
    public double[] _joints = new double[6];
    public static readonly string[] LinkNames ={"world/dummy_link/base_link/shoulder_link",
                                                "/upper_arm_link", 
                                                "/forearm_link", 
                                                "/wrist_1_link",  
                                                "/wrist_2_link",  
                                                "/wrist_3_link"};
    
    public static string EntryLinkName = "/tool0/robotiq_coupler/robotiq_85_base_link";
    public static readonly string[] EndEffectorLinkNames_Right = {"/robotiq_85_right_inner_knuckle_link",
                                                                  "/robotiq_85_right_inner_knuckle_link/robotiq_85_right_finger_tip_link", 
                                                                  "/robotiq_85_right_knuckle_link"};

    public static readonly string[] EndEffectorLinkNames_Left  = {"/robotiq_85_left_inner_knuckle_link",  
                                                                  "/robotiq_85_left_inner_knuckle_link/robotiq_85_left_finger_tip_link", 
                                                                  "/robotiq_85_left_knuckle_link"};
    ArticulationBody[] m_ur5ArticulationBody;

    ArticulationBody[] m_robotiqGripperR;
    ArticulationBody[] m_robotiqGripperL;

    public float p_gain = 6.0f;//5.0f;
    public float i_gain = 0.1f;//0.8f;
    public float d_gain = 1.0f;//1.0f;
    public float m_i_max =  0.7f;//(float)Math.PI;
    public float m_i_min = -0.7f;//(float)Math.PI;
    public float[] p_istate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    public float[] p_dstate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    public float   p_gain_eel   = 5.0f;
    public float   i_gain_eel   = 1.8f;
    public float   d_gain_eel   = 1.0f;
    public float   m_i_max_eel  = (float)Math.PI;
    public float   m_i_min_eel  = -(float)Math.PI;
    public float[] p_istate_eel = {0.0f, 0.0f, 0.0f};
    public float[] p_dstate_eel = {0.0f, 0.0f, 0.0f};


    public float   p_gain_eer   = 5.0f;
    public float   i_gain_eer   = 1.8f;
    public float   d_gain_eer   = 1.0f;
    public float   m_i_max_eer  = (float)Math.PI;
    public float   m_i_min_eer  = -(float)Math.PI;
    public float[] p_istate_eer = {0.0f, 0.0f, 0.0f};
    public float[] p_dstate_eer = {0.0f, 0.0f, 0.0f};

    private float curr_j = 0.0f;
    
    public float gripper_goal = 0.0f;


    void Start()
    {   
        publishMessageTime = (1/publisheFreq);
        // find specific game object
        ur5 = GameObject.Find("ur5_1");
        // ros subscriber 
        // ROSConnection.GetOrCreateInstance().Subscribe<Ur5JointsMsg>("real_ur5/joints", jointCallback);
        ROSConnection.GetOrCreateInstance().Subscribe<Ur5JointsMsg>("ur5_shadow/joints", jointCallback);
        
        // load articulate body with non static joints
        var link_names              = string.Empty;
        m_ur5ArticulationBody       = new ArticulationBody[num_joints];
        for (var i = 0; i < num_joints; i++){
            link_names += LinkNames[i];
            // Debug.Log(link_names);

            m_ur5ArticulationBody[i]       = ur5.transform.Find(link_names).GetComponent<ArticulationBody>();
            // Debug.Log(string.Format("Articulate body: {0} and joint position: {1}",m_ur5ArticulationBody[i],m_ur5ArticulationBody[i].jointPosition[0]));
        }

        // load articulate body of non static joints of robotiq
        m_robotiqGripperL = new ArticulationBody[ee_joints];
        m_robotiqGripperR = new ArticulationBody[ee_joints];

        for (var i = 0; i < ee_joints ; i++){
            var eeLeftLinkName = link_names + EntryLinkName + EndEffectorLinkNames_Left[i];
            var eeRightLinkName = link_names + EntryLinkName + EndEffectorLinkNames_Right[i];
            m_robotiqGripperL[i] = ur5.transform.Find(eeLeftLinkName).GetComponent<ArticulationBody>();
            m_robotiqGripperR[i] = ur5.transform.Find(eeRightLinkName).GetComponent<ArticulationBody>();
        }

    }
    
    // callback for joint angles published over specified ros topic
    void jointCallback(Ur5JointsMsg JointMsg)
    {
        for (var i = 0 ; i<num_joints; i++){
            _joints[i] = JointMsg.joints[i];
        }
    }

    
    void applyControlRobotiq(ArticulationBody[] RobotiqBody, string ee_type){
        
        for(var i = 0; i < ee_joints; i++){
            var eeDrive = RobotiqBody[i].xDrive;

            if (i == 1){
                var error = (float)(-gripper_goal * Math.PI/ 180) - (float)RobotiqBody[i].jointPosition[0];
                // Debug.Log(string.Format("Robotiq Debug: {0}",RobotiqBody[i].jointPosition[0]));
                
                // apply pid controller
                if (ee_type == "left"){
                // avoid integral winding                  
                    p_istate_eel[i] += (float)error;
                    if (p_istate_eel[i] > m_i_max){
                        p_istate_eel[i] = m_i_max;
                    }
                    else if (p_istate_eel[i] < m_i_min){
                        p_istate_eel[i] = m_i_min;
                    }
                    var velocity  = (p_gain_eel * error) + (p_istate_eel[i] * i_gain_eel) + ((error - p_dstate_eel[i]) * d_gain_eel);
                    p_dstate_eel[i] = (float)error;
                    eeDrive.damping = 1e+18f;
                    eeDrive.stiffness = 1e+18f;
                    eeDrive.forceLimit = 1e+20f;
                    eeDrive.target = (float)(-gripper_goal);
                    eeDrive.targetVelocity = (float)(velocity * 180 / Math.PI);
                    RobotiqBody[i].xDrive = eeDrive;
                }

                else if (ee_type == "right"){
                // avoid integral winding                  
                    p_istate_eer[i] += (float)error;
                    if (p_istate_eer[i] > m_i_max){
                        p_istate_eer[i] = m_i_max;
                    }
                    else if (p_istate_eer[i] < m_i_min){
                        p_istate_eer[i] = m_i_min;
                    }
                    var velocity  = (p_gain_eer * error) + (p_istate_eer[i] * i_gain_eer) + ((error - p_dstate_eer[i]) * d_gain_eer);
                    p_dstate_eel[i] = (float)error;
                    eeDrive.damping = 1e+18f;
                    eeDrive.stiffness = 1e+18f;
                    eeDrive.forceLimit = 1e+20f;
                    eeDrive.target = (float)(gripper_goal);
                    eeDrive.targetVelocity = (float)(velocity * 180 / Math.PI);
                    RobotiqBody[i].xDrive = eeDrive;
                }

            }

            else{

                // Debug.Log(string.Format("Robotiq Debug: {0}",RobotiqBody[i].jointPosition[0]));
                var error = (float)(gripper_goal * Math.PI/ 180) - (float)RobotiqBody[i].jointPosition[0];
                
                // apply pid controller
                if (ee_type == "left"){
                // avoid integral winding                  
                    p_istate_eel[i] += (float)error;
                    if (p_istate_eel[i] > m_i_max){
                        p_istate_eel[i] = m_i_max;
                    }
                    else if (p_istate_eel[i] < m_i_min){
                        p_istate_eel[i] = m_i_min;
                    }
                    var velocity  = (p_gain_eel * error) + (p_istate_eel[i] * i_gain_eel) + ((error - p_dstate_eel[i]) * d_gain_eel);
                    p_dstate_eel[i] = (float)error;
                    eeDrive.damping = 1e+18f;
                    eeDrive.stiffness = 1e+18f;
                    eeDrive.forceLimit = 1e+20f;
                    eeDrive.target = (float)(gripper_goal);
                    eeDrive.targetVelocity = (float)(velocity * 180 / Math.PI);
                    RobotiqBody[i].xDrive = eeDrive;
                }

                else if (ee_type == "right"){
                // avoid integral winding                  
                    p_istate_eer[i] += (float)error;
                    if (p_istate_eer[i] > m_i_max){
                        p_istate_eer[i] = m_i_max;
                    }
                    else if (p_istate_eer[i] < m_i_min){
                        p_istate_eer[i] = m_i_min;
                    }
                    var velocity  = (p_gain_eer * error) + (p_istate_eer[i] * i_gain_eer) + ((error - p_dstate_eer[i]) * d_gain_eer);
                    p_dstate_eel[i] = (float)error;
                    eeDrive.damping = 1e+18f;
                    eeDrive.stiffness = 1e+18f;
                    eeDrive.forceLimit = 1e+20f;
                    eeDrive.target = (float)(gripper_goal);
                    eeDrive.targetVelocity = (float)(velocity * 180 / Math.PI);
                    RobotiqBody[i].xDrive = eeDrive;
                }

            }


           
        }
    }

    void applyControlUR5(ArticulationBody[] ur5_body){
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
                currentJDrive.damping = 1e+10f;
                currentJDrive.stiffness = 1e+1f;
                currentJDrive.forceLimit = 1e+12f;
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
    
    void Update(){
        
        timeElapsed1 += Time.deltaTime;
        publishMessageTime = (1/publisheFreq);
        if (timeElapsed1 > publishMessageTime)
        {   
            applyControlUR5(m_ur5ArticulationBody);
            applyControlRobotiq(m_robotiqGripperL, "left");
            applyControlRobotiq(m_robotiqGripperR, "right");
            timeElapsed1 = 0;
        }
    }
}