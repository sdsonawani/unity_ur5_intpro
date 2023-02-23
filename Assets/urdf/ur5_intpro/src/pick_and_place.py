#! /usr/bin/env python3


import rospy
from irl_robots.msg import ur5Joints
import time
import numpy as np
import random

CLOSING_DISTANCE = 0 #105
OPENING_DISTANCE = 0


def pick_and_place_object(object_id=0, flag=True):
    rospy.set_param("grasp_distance",OPENING_DISTANCE)
    rospy.set_param("go_over_object",False)
    rospy.set_param("place_object",False)
    rospy.set_param("go_to_object",False)
    rospy.set_param("go_to_midhome",False)
    rospy.set_param("finished_traj",False)
    rospy.set_param("finished_subtask",False)
    rospy.set_param("go_final",False)
    

    # try:
    
    # Go over the object
    rospy.set_param("go_over_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        if flag:
            print("waiting to finish traj for going over the object")
            flag = False
            
        time.sleep(0.1)
    rospy.set_param("go_over_object",False)
    rospy.set_param("finished_traj",False)
    flag = True


    # Go to the object
    rospy.set_param("go_to_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        if flag:
            print("waiting to finish traj for going to object")
            flag = False
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_to_object",False)
    flag = True

    # Grasp the object
    rospy.set_param("grasp_distance",CLOSING_DISTANCE)

    # Go overt the object
    rospy.set_param("go_over_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        if flag:
            print("waiting to finish traj for going over the object")
            flag = False
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_over_object",False)
    flag = True
    
    # Go the final way point
    rospy.set_param("go_final",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        if flag:
            print("waiting to finish traj for final way_point")
            flag = False
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_final",False)
    flag = True
    
    # Place the object on the ground
    rospy.set_param("place_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        if flag:
            print("waiting to finish traj for going to place the object")
            flag = False
        time.sleep(0.1)            
    rospy.set_param("finished_traj",False)
    rospy.set_param("place_object",False)
    flag = True
    
    
    # # Slide the object back
    # rospy.set_param("slide_object",True)
    # rospy.set_param("move_to_next_primitive",True)
    # while not rospy.get_param("finished_traj"):
    #     if flag:
    #         print("waiting to finish traj for sliding the object")
    #         flag = False
    #     time.sleep(0.1)
    # rospy.set_param("finished_traj",False)
    rospy.set_param("slide_object",False)
    # flag = True
    
    # Open the gripper
    rospy.set_param("grasp_distance",OPENING_DISTANCE)
    
    # Go to the final way point
    rospy.set_param("go_final",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        if flag:
            print("waiting to finish traj for final way_point")
            flag = False
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_final",False)
    
        
    rospy.set_param("finished_subtask",True)
    # except KeyboardInterrupt:
    #     print("Setting default values to ros parameters")
    #     rospy.set_param("go_over_object",False)
    #     rospy.set_param("finished_traj",True)
    #     rospy.set_param("place_object",False)
    #     rospy.set_param("go_to_object",False)
        
    
# {0:[0.6, 0.0, 0.08],
# 1:[0.6, 0.2, 0.08],
# 2:[0.6, 0.4, 0.08],
# 3:[0.6,-0.2, 0.08],
# 4:[0.6,-0.4, 0.08],
# 5:[0.35, 0.0, 0.08],
# 6:[0.35, 0.2, 0.08],
# 7:[0.35, 0.4, 0.08],
# 8:[0.35,-0.2, 0.08],
# 9:[0.35,-0.4, 0.08]}

if __name__ == '__main__':
    
    rospy.init_node("pick_and_place",anonymous=True)
    # goal_id = [0,1,2,3,4,5,6,7,8,9]
    
    # zig zag [W-shape]
    # goal_id = [7,1,5,3,9]
    
    # zig zag [M-shape]
    # goal_id = [2,6,0,8,4]
    goal_id = [2,6,0,8]
    
    # vertical 3I
    # goal_id = [7,2,5,0,9,4]
    
    # vertical 2I
    # goal_id = [6,1,8,3]
    

    # goal_id = [1,2]

    # random.shuffle(goal_id)
    # goal_id = goal_id[0:int(len(goal_id)/3)]
    
    print("Going to sort total [{}] objects".format(len(goal_id)))
    try:
        for id in goal_id:
            print("Going to id {}".format(id))
            rospy.set_param("goal_id",id)
            time.sleep(0.1)
            pick_and_place_object(object_id=id)    
    except KeyboardInterrupt:
        print("Shuting down pick and place")