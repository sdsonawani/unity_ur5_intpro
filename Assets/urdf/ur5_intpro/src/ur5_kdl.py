#! /usr/bin/env python3

from xml.etree.ElementTree import parse
import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl
from sensor_msgs.msg import JointState, Joy
from transforms3d.affines import compose
from transforms3d.euler import mat2euler, euler2mat
from transforms3d.quaternions import quat2mat , mat2quat
import threading
import os
import sys
import time
import threading
import argparse
from six.moves import queue
import serial
from ds4_mapper import DS4_Mapper
from utils import ur5_intpro_utils

class Custom_UR5_KDL:
    '''
    Documents:
        1)
        2)
    '''
    def __init__(self, pkg_name="ur5_intpro", init_delay=0.1, go_midhome=True):
        
        rospy.init_node("ur5_kdl")
        
        self.this_pkg_path = rospkg.RosPack().get_path(pkg_name)
        self.rate = 5
        
        # general params
        self.general_params = ur5_intpro_utils.load_yaml(os.path.join(self.this_pkg_path,"config","general_params.yaml"))
        
        # init ros msgs
        self.joint_state_msg = JointState()
        self.joy_msg         = Joy()
        self.ur5_control_msg = ur5Control()
        self.ur5_matrix      = matrix()
        self.ur5_rows        = rows()
        self.ur5_rows.values = []
        self.ur5_matrix.rows = self.ur5_rows
        self.ur5_joints      = ur5Joints()
        self.r2fg_msg        = gSimpleControl()
        
        self.ur5_control_msg.command      = rospy.get_param("irl_robot_command",  default="movej") 
        self.ur5_control_msg.acceleration = rospy.get_param("irl_robot_accl",     default=np.pi/2)
        self.ur5_control_msg.velocity     = rospy.get_param("irl_robot_vel",      default=np.pi/2) 
        self.ur5_control_msg.time         = rospy.get_param("irl_robot_com_time", default=0)
        self.ur5_control_msg.jointcontrol = True
        self.ur5_control_timeout = 0

        rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,buff_size=1)
        rospy.Subscriber("/ur5/joints",ur5Joints,self.ur5_joints_callback)
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        
        
        # ds4 key mapper
        self.ds4_mapper = DS4_Mapper()
        self.enable_ds4 = False
        self.limit_cart = self.general_params["limit_cart"]
        self.cart_step  = self.general_params["cart_step"]
        
        # kdl setup 
        self.urdf_path = os.path.join(self.this_pkg_path,"urdf",self.general_params["kdl_urdf"])
        self.ur5_tf_chain = self.init_kdl()
        self.ur5_ik_solver =  KDL.ChainIkSolverPos_LMA(self.ur5_tf_chain, 1e-8 ,1000, 1e-6)
        # self.ur5_ik_solver =  KDL.ChainIkSolverPos_NR(Chain=self.ur5_tf_chain)
        
        # ik prameters
        self.goal_x   = self.general_params["ik_goal"]["x"]
        self.goal_y   = self.general_params["ik_goal"]["y"]
        self.goal_z   = self.general_params["ik_goal"]["z"]
        self.yaw      = np.deg2rad(self.general_params["ik_goal"]["yaw"])
        self.pitch    = np.deg2rad(self.general_params["ik_goal"]["pitch"])
        self.roll     = np.deg2rad(self.general_params["ik_goal"]["roll"])
        # for vertical ee config keep yaw=0.0 pitch=np.pi and roll=0.0
        # for horizontal ee  config keep yaw=np.pi/2.0 pitch=0.0 roll=np.pi/2.0
        # self.yaw      = np.pi/2.0
        # self.pitch    = 0.0
        # self.roll     = np.pi/2.0    
        self.rot_cont        = True
        self.z_offset        = 0.0
        self.grasp_value     = 0
        self.sim_grasp_value = 0.0
        self.grasp_steps     = 8
        
        self.ur5_joint_publisher        = [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
        self.real_ur5_joint_publisher   = rospy.Publisher("/ur5/control",ur5Control,queue_size=10)
        self.r2fg_control_publisher     = rospy.Publisher("/r2fg/simplecontrol",gSimpleControl,queue_size=10)
        self.sim_r2fg_control_publisher = rospy.Publisher("/finger_position_controller/command",Float64,queue_size=10)
        # self.real_ur5_joint_publisher   = rospy.Publisher("/ur5/continuous_controller",matrix,queue_size=10)
                
        self.pub_real_ur5      = False
        self.pub_real_ur5_cont = False
             
        rospy.sleep(init_delay)
        rospy.set_param("finished_subtask",True)
        rospy.set_param("grasp_distance",0)
        rospy.set_param("slide_object",False)
        rospy.set_param("move_to_next_primitive",False)
        
        self.goals = self.general_params["goals"]
        rospy.set_param("goal_id",0)
        if go_midhome:
            self.init_midhome()
    
    def __del__(self):
        print("going home")
        joints = [0, -1.57, 0 , -1.57, 0, 0] 
        tmp = 0
        while tmp<1000:
            for i, publisher in enumerate(self.ur5_joint_publisher):
                publisher.publish(joints[i])
                print("homing joint")
                rospy.sleep(0.1)
            tmp += 1
    
    def ur5_joints_callback(self,msg):
        self.ur5_joints = msg
             
    def joint_state_callback(self,msg):
        self.joint_state_msg = msg
    
    def joy_callback(self,msg):
        self.joy_msg = msg 
         
    def start_ros_spin(self):
        print("Starting ros spin")
        rospy.spin()
    
    def init_kdl(self,root_link="base_link",leaf_link="tool0"):
        kdl_tree = kdl_parser_py.urdf.treeFromFile(self.urdf_path)[1]
        return kdl_tree.getChain(root_link,leaf_link)
    

    def init_midhome(self):
        # self.goal_x   = 0.2
        # self.goal_y   = 0.4
        # self.goal_z   = 0.35
        # self.yaw      = np.pi/2.
        # self.pitch    = 0
        # self.roll     = np.pi/2.0
        # self.rot_cont = True
        # midhome_joints = self.get_ik()
        midhome_joints = [1.6459543704986572, -1.6812246481524866, 1.4999432563781738, -1.3898146788226526, -1.5704978148089808, 0.8604261875152588]
        self.ur5_control_msg.values = midhome_joints
        self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
        for i, publisher in enumerate(self.ur5_joint_publisher):
            publisher.publish(midhome_joints[i])
            rospy.sleep(0.1)
        rospy.sleep(1)
        return True
    
    def home(self):
        joints = [0, -1.57, 0 , -1.57, 0, 0] 
        tmp = 0
        # while tmp<10:
        for i, publisher in enumerate(self.ur5_joint_publisher):
            publisher.publish(joints[i])
            print("homing joint")
            rospy.sleep(0.1)
            # tmp += 1
    
    def grasp(self, value, force=255, speed = 255):
        if value==None:
            print("Please provide gripper closing distance")
            return
        if value > 0:
            print("Closing distance: {}".format(value))
        
        self.sim_grasp_value = float(value/255)
        self.r2fg_msg.position = abs(int(value))
        self.r2fg_msg.force    = abs(int(force))
        self.r2fg_msg.speed    = abs(int(speed))
        self.r2fg_control_publisher.publish(self.r2fg_msg)
        self.sim_r2fg_control_publisher.publish(self.grasp_value)
        # if value
        # rospy.sleep(0.5)
            
    def get_ik(self, nums_joints=6, enable_rot=False, yaw_offset=0):
        
        # get goal id (only for testing)
        goal_id = rospy.get_param("goal_id")
        kdl_init_joints = KDL.JntArray(nums_joints)
        kdl_init_joints[0] = self.joint_state_msg.position[3] 
        kdl_init_joints[1] = self.joint_state_msg.position[2] 
        kdl_init_joints[2] = self.joint_state_msg.position[0]
        kdl_init_joints[3] = self.joint_state_msg.position[4]
        kdl_init_joints[4] = self.joint_state_msg.position[5]
        kdl_init_joints[5] = self.joint_state_msg.position[6] 
        
        if not rospy.get_param("finished_subtask"):
            current_goal =  self.goals[goal_id]
            
            if rospy.get_param("go_over_object"):
                print("Going over object")
                self.goal_x = current_goal[0] #0.6
                self.goal_y = current_goal[1] #0.0
                self.goal_z = 0.25 #0.25
                self.ur5_control_msg.time = 1

            
            elif rospy.get_param("go_to_object") and not rospy.get_param("go_over_object"):
                print("Going to object")
                self.goal_x = current_goal[0]#0.6
                self.goal_y = current_goal[1]#0.0
                self.goal_z = current_goal[2]#0.1
                self.ur5_control_msg.time = 1
                
            elif rospy.get_param("go_to_midhome"):
                print("Going to midhome")
                self.goal_x = 0.2
                self.goal_y = 0.0
                self.goal_z = 0.5
            
            elif rospy.get_param("go_final"):
                print("Going to final waypoint")

                self.goal_x = 0.2
                self.goal_y = 0.4
                if current_goal[1]>0:
                    self.goal_y = 0.4
                else:
                    self.goal_y = -0.4
                self.goal_z = 0.25
                self.ur5_control_msg.time = 1
                
            elif rospy.get_param("place_object"):
                print("Placing the object")
                # self.goal_x = -0.45
                self.goal_x = -0.45
                # self.goal_y = 0.35
                
                if current_goal[1]>0:
                    self.goal_y = 0.35
                else:
                    self.goal_y = -0.35
                    
                self.goal_z = 0.10
                # self.ur5_control_msg.time = 1

                
            elif rospy.get_param("slide_object"):
                print("Sliding the object")
                # self.goal_x = -0.45
                self.goal_x = -0.1
                # self.goal_y = 0.35
                
                if current_goal[1]>0:
                    self.goal_y = 0.35
                else:
                    self.goal_y = -0.35
                    
                self.goal_z = 0.10
        # else:
        #     self.goal_x = 0.4
        #     self.goal_y = 0.0
        #     self.goal_z = 0.6
            
        if yaw_offset:
            print("Using yaw offset")
            tf = compose([0,0,0],euler2mat(0,0,yaw_offset),[1,1,1])
            self.z_offset = self.goal_z + 0.15
            goal_tf = compose([self.goal_x,self.goal_y,self.z_offset],euler2mat(self.yaw,self.pitch,self.roll),[1,1,1])
            goal_tf = np.matmul(tf,goal_tf)
            # print(goal_tf[0:3,-1])
            just_xyz = KDL.Vector(goal_tf[0,-1],goal_tf[1,-1],goal_tf[2,-1])
            rot = mat2euler(goal_tf[0:3,0:3])
            just_rpy = KDL.Rotation().EulerZYX(rot[2],rot[1],rot[0]) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
            
        else:   
            self.z_offset = self.goal_z + 0.15
            
            just_xyz = KDL.Vector(self.goal_x, self.goal_y, self.z_offset)     
            just_rpy = KDL.Rotation().EulerZYX(self.roll, self.pitch, self.yaw) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
                
        kdl_goal_frame = KDL.Frame(just_rpy,just_xyz)
        kdl_goal_joints = KDL.JntArray(nums_joints)
        self.ur5_ik_solver.CartToJnt(kdl_init_joints, kdl_goal_frame, kdl_goal_joints)
        return kdl_goal_joints
        
    def ur5_publisher(self, joints, delay = 0.1, traj_time = 1):

        if rospy.get_param("move_to_next_primitive"):
            self.ur5_control_timeout = 0
        
        current_ur5_joints = [jnt for jnt in self.ur5_joints.positions]
        goal_joints        = [jnt for jnt  in joints]
        
        
        if not np.allclose(current_ur5_joints,goal_joints, atol=1e-2):
            rospy.set_param("finished_traj",False)
            print("Trajectory_not_finished!")
        else:
            rospy.set_param("finished_traj",True)
            print("Finished trajectory")
        
        
        if self.pub_real_ur5:
            print("!!!!! Publishing on real ur5 robot !!!!!")
            if not rospy.get_param("finished_traj"):
                self.ur5_control_msg.values = joints
                rospy.sleep(rospy.get_param("shadow_delay",default=0.5))
                self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                rospy.set_param("move_to_next_primitive",False)
                self.ur5_control_timeout = 0


        self.ur5_control_timeout += 1
        print("current time out {}".format(self.ur5_control_timeout))
        
       

        if self.ur5_control_timeout > self.rate:
            print("Timeout Ouccured!!!!1\n")
            rospy.set_param("finished_traj",True)
            self.ur5_control_timeout = 0
              
        for i, publisher in enumerate(self.ur5_joint_publisher):
            publisher.publish(joints[i])
            rospy.sleep(delay)
                
        self.pub_real_ur5 = rospy.get_param("move_to_next_primitive",default=False)
        return 
    
    def ds4_ik_mapper(self):
        if self.joy_msg.buttons[self.ds4_mapper.box] == 1:
            self.goal_y +=  self.cart_step
            if self.goal_y > self.limit_cart:
                self.goal_y = self.limit_cart
    
        if self.joy_msg.buttons[self.ds4_mapper.circle] == 1:
            self.goal_y -=  self.cart_step
            if self.goal_y < -self.limit_cart:
                self.goal_y = -self.limit_cart
    
        if self.joy_msg.buttons[self.ds4_mapper.l1] == 1:
            if self.joy_msg.buttons[self.ds4_mapper.triangle] == 1:
                self.goal_z += self.cart_step
                if self.goal_z > self.limit_cart:
                    self.goal_z = self.limit_cart
            
            if self.joy_msg.buttons[self.ds4_mapper.cross] == 1:
                self.goal_z -= self.cart_step
                if self.goal_z < -self.limit_cart:
                    self.goal_z = -self.limit_cart
        else:
            if self.joy_msg.buttons[self.ds4_mapper.triangle] == 1:
                self.goal_x +=  self.cart_step
                if self.goal_x > self.limit_cart:
                    self.goal_x = self.limit_cart
        
            if self.joy_msg.buttons[self.ds4_mapper.cross] == 1:
                self.goal_x -=  self.cart_step
                if self.goal_x < -self.limit_cart:
                    self.goal_x = -self.limit_cart
        
        if self.joy_msg.buttons[self.ds4_mapper.l3] == 1:
            self.roll  = 0.0
            self.pitch = np.pi 
            self.yaw   = 0.0 
        
        
        if self.joy_msg.buttons[self.ds4_mapper.l2] ==  1:
            self.grasp_value -= self.grasp_steps
            if self.grasp_value < 1:
                self.grgrasp_valueap_value = 0
                
        if self.joy_msg.buttons[self.ds4_mapper.r2] ==  1:
            self.grasp_value += self.grasp_steps
            if self.grasp_value > 255:
                self.grasp_value = 255
              
        
        if self.joy_msg.buttons[self.ds4_mapper.l1] == 1 and self.joy_msg.buttons[self.ds4_mapper.r1] == 1:
            if self.pub_real_ur5:
                self.pub_real_ur5 = False
            else:
                self.pub_real_ur5 = True
            print("Real robot control {}".format(self.pub_real_ur5))

        if self.joy_msg.buttons[self.ds4_mapper.l2] == 1 and self.joy_msg.buttons[self.ds4_mapper.r2] == 1:
            if self.pub_real_ur5_cont:
                self.pub_real_ur5_cont = False
            else:
                self.pub_real_ur5_cont= True
            print("Real robot control {}".format(self.pub_real_ur5))
            
        if self.joy_msg.buttons[self.ds4_mapper.ps] == 1 and self.joy_msg.buttons[self.ds4_mapper.option] == 1:
            print("Going to mid home!!!!")
            self.init_midhome()
            rospy.sleep(1)
            self.goal_x   = 0.2
            self.goal_y   = 0.4
            self.goal_z   = 0.35
        
            
    def ds4_teleop(self):
        try:
            if self.joy_msg.buttons[self.ds4_mapper.ps] == 1:
                self.enable_ds4 = True
                rospy.sleep(0.5)
            if self.enable_ds4:
                print("\n ==== Enabled ds4 teleop ==== \n")
                self.ds4_ik_mapper()
                if self.joy_msg.buttons[self.ds4_mapper.ps] == 1:
                    self.enable_ds4 = False
                    rospy.sleep(0.5)
            
        except IndexError:
            print("Looks ds4 controller is not on!!")
            
        
    
if __name__ == '__main__':
    
    ur5_kdl = Custom_UR5_KDL(go_midhome=False)
    rate    = rospy.Rate(ur5_kdl.rate)
    while not rospy.is_shutdown():
        try:
            print("running node")  
            kdl_goal_joints = ur5_kdl.get_ik(enable_rot=True)
            
            # if rospy.get_param("use_ds4_gripper",default=False):
            rospy.set_param("grasp_distance",float(ur5_kdl.grasp_value))
            
            ur5_kdl.grasp(value=rospy.get_param("grasp_distance"))
            print("cartesian goals: {}".format([ur5_kdl.goal_x, ur5_kdl.goal_y, ur5_kdl.goal_z]))
            ur5_kdl.ur5_publisher(joints=kdl_goal_joints, delay=0.0)
            ur5_kdl.ds4_teleop()
            rate.sleep()
            
        except KeyboardInterrupt:
            print("shutting down!")
        
       