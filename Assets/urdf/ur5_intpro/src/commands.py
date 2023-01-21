from re import L
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random
import numpy as np
import cv2
import rospkg
import os
import time
import argparse
import threading

def generate_pattern(goal_id, num_robot_goals = 6):
    pattern = 255*np.ones((340,670,3),dtype="uint8")
    robot_goals = np.random.choice(goal_id.reshape(-1),size=num_robot_goals,replace=False)    
    rsize = 100
    for i,goal in enumerate(goal_id):
        for j,pose in enumerate(goal):
            x11 = 10 + j * 110
            y11 = 10 + i*110
            points = np.array([[x11,y11],[x11+rsize,y11], [x11+rsize, y11+rsize], [x11, y11+rsize]], dtype=np.int32)
            if pose in robot_goals:
                pattern = cv2.fillConvexPoly(pattern, points, (0,0,255))
            else:
                pattern = cv2.fillConvexPoly(pattern, points, (0,255,0))
            cv2.putText(pattern,str(pose),(x11+int(rsize/2), y11+int(rsize/2)),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),thickness=2)

    return pattern, robot_goals


def save_data(pattern_img, robot_goals, time_elapsed, exp_type, subject_name):
    if not os.path.isdir(os.path.join(ur5_intpro_dir,"data",args.subject_name)):
                os.mkdir(os.path.join(ur5_intpro_dir,"data",args.subject_name))

    t = time.localtime()
    timestamp = time.strftime('-%b-%d-%Y_%H_%M_%S', t)       
    pattern_filename     = os.path.join(ur5_intpro_dir,"data",subject_name, exp_type + "-pattern"+timestamp+".png")
    robot_goals_filename = os.path.join(ur5_intpro_dir,"data",subject_name, exp_type + "-robot_goals"+timestamp+".txt")
    time_filename = os.path.join(ur5_intpro_dir,"data",subject_name, exp_type + "-time_elapsed"+timestamp+".txt")

    cv2.imwrite(pattern_filename,pattern)
    np.savetxt(robot_goals_filename,robot_goals,comments="Robot goal ids")
    np.savetxt(time_filename,np.array([time_elapsed]),comments="Execution time for sub experiment (nano seconds)")

# def counter():
#     for

if __name__ == '__main__':


    parser =  argparse.ArgumentParser()
    parser.add_argument("--subject_name","-sn",type=str,default="shubham")
    parser.add_argument("--delay","-d",type=float,default="0.2")
    args = parser.parse_args()
    
    rospy.init_node("testing")

    global img_publisher
    img_publisher = rospy.Publisher("pattern_image",Image,queue_size=1)
    global ur5_intpro_dir
    ur5_intpro_dir = rospkg.RosPack().get_path("ur5_intpro")
    goal_id = np.array([[5,4,3,2,1,0],
                        [11,10,9,8,7,6],
                        [17,16,15,14,13,12]],dtype=np.int)

    np.random.seed(0)    
    pattern, robot_goals = generate_pattern(goal_id)
    
    global t1
    t1 = 0.0
    global exp_type
    
    while not rospy.is_shutdown():
        try:
            cv2.imshow("pattern",pattern)

            img_msg = CvBridge().cv2_to_imgmsg(pattern,encoding="bgr8")
            img_publisher.publish(img_msg)

            key = cv2.waitKey(1)
            if key == ord('n'):
                pattern, robot_goals = generate_pattern(goal_id)

            if key == ord('0'):
                rospy.set_param("do_render",False)
                rospy.set_param("show_shadow",False)
                rospy.set_param("render_all",False)

            
            if key == ord('1'):
                rospy.set_param("do_render",True)
                rospy.set_param("render_all",True)

            
            if key == ord('2'):
                rospy.set_param("do_render",True)
                rospy.set_param("show_shadow",True)

            if rospy.get_param("show_shadow",default=False) and rospy.get_param("render_all",default=False) and rospy.get_param("do_render",default=False):
                exp_type = "dual_mode"

            elif rospy.get_param("show_shadow",default=False) and not rospy.get_param("render_all",default=False) and rospy.get_param("do_render",default=False):
                exp_type = "shadow_mode"

            elif not rospy.get_param("show_shadow",default=False) and rospy.get_param("render_all",default=False) and rospy.get_param("do_render",default=False):
                exp_type = "highlight_mode"
            
            elif not rospy.get_param("do_render",default=False):
                exp_type = "no_mode"

            if key == ord('g'):
                if rospy.get_param("grasp_distance",default=0) == 105:
                    rospy.set_param("grasp_distance",0)
                    print("Setting grasp distance to 0")
                    rospy.sleep(1)
                else:
                    rospy.set_param("grasp_distance",105)
                    print("Setting grasp distance to 105")
                    rospy.sleep(1)

            if key == ord('a'):
                rospy.set_param("goal_ids",[int(i) for i in goal_id.reshape(-1)])
                

            if key == ord('h'):
                rospy.set_param("go_home",True)

            if key == ord("s"):
                t1 = rospy.Time().now().to_nsec()
                rospy.set_param("delay",args.delay)
                rospy.set_param("goal_ids",[int(g) for g in robot_goals])
                for goal in robot_goals:
                    rospy.set_param("goal_id",int(goal))
                    rospy.set_param("move_to_next_primitive",True)
                    tmp = 0
                    while rospy.get_param("move_to_next_primitive"):
                        if tmp > 0:
                            continue
                        else:
                            print("waiting\r")
                            tmp += 1
        
            if rospy.get_param("start_user_exp",default=False):
                print("Listened to rosparam over server, Starting experiment!!!")
                t1 = rospy.Time().now().to_nsec()
                rospy.set_param("delay",args.delay)
                rospy.set_param("goal_ids",[int(g) for g in robot_goals])
                for goal in robot_goals:
                    rospy.set_param("goal_id",int(goal))
                    rospy.set_param("move_to_next_primitive",True)
                    tmp = 0
                    while rospy.get_param("move_to_next_primitive"):
                        if tmp > 0:
                            continue
                        else:
                            print("waiting\r")
                            tmp += 1
                rospy.set_param("start_user_exp",False)
            
            if rospy.get_param("stop_user_exp",default=False):
                t2 = rospy.Time().now().to_nsec()
                global time_elapsed
                time_elapsed = t2 - t1
                save_data(pattern, robot_goals, t2 - t1, exp_type, subject_name=args.subject_name)
                print("Total_time elapsed in experiment: {}".format(t2-t1))
                rospy.set_param("stop_user_exp",False)
                
            # if key == ord('f'):
            #     t2 = rospy.Time().now().to_nsec()
            #     time_elapsed = t2 - t1
            #     save_data(pattern, robot_goals, time_elapsed, exp_type, subject_name=args.subject_name)
            #     print("Total_time elapsed in experiment: {}".format(t2-t1))

            if key == ord('q'):
                break
            rospy.sleep(0.01)
        except  KeyboardInterrupt or CvBridgeError:
            print("Shutting down!")