#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os
import rospkg


class USER_INT:
    def __init__(self) -> None:
        rospy.init_node("user_interface")
        rospy.set_param("start_user_exp",False)
        rospy.set_param("stop_user_exp",False)
        rospy.set_param("go_home",False)
        
        self.user_interface = Image()
        rospy.Subscriber("/user_display",Image,self.imageCallback)
    
    def imageCallback(self, msg) -> None:
        self.user_interface = msg
 
    def run_experiment(self):
        try: 
            if not self.user_interface.data:
                print("Did not receive msg...")
                return

            print("wating for input...")
            cv_image = CvBridge().imgmsg_to_cv2(self.user_interface,desired_encoding="bgr8")
            cv2.imshow("Experiment",cv2.resize(cv_image,(1000,500))) 
            cv_key = cv2.waitKey(1)
            if cv_key == 32: # space
                print("Stopping experiment")
                rospy.set_param("stop_user_exp",True)
                rospy.set_param("start_user_exp",False)
            
            elif cv_key == 13: # enter
                print("Starting experiment")
                rospy.set_param("start_user_exp",True)
                rospy.set_param("stop_user_exp",False)
            
            elif cv_key == ord('h'):
                rospy.set_param("go_home",True)
                
            if cv_key == ord('n'):
                # rospy.set_param("reinit_goals",True)
                rospy.set_param("new_mode",True)
        
        except Exception as e:
            print(f"Run error: {e}")

if __name__ == '__main__':
    
    ui  =  USER_INT()
    while not rospy.is_shutdown():
        ui.run_experiment()
        rospy.sleep(0.01)