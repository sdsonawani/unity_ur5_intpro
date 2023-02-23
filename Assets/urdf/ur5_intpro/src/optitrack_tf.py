#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from transforms3d.affines import compose
from transforms3d.euler import euler2mat
from transforms3d.quaternions import quat2mat, mat2quat
import numpy as np

def publish_pose(msg, publisher):
    trans_world = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
    rot_world   = quat2mat([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    tf = compose(trans_world,rot_world,[1,1,1])
    trans_stat = [0, 0, 0]
    # trans_stat = [2.3, 0.05, -0.95]
    rot_stat   = euler2mat(np.pi/2.,0,np.pi/2)
    rot_stat1   = euler2mat(-np.pi/2.,0,0)
    
    tf2  = compose(trans_stat,rot_stat,[1,1,1])
    tf3  = compose([0,0,0],rot_stat1,[1,1,1])
    final_tf = np.matmul(tf2,tf)
    final_tf = np.matmul(final_tf,tf3)
    pose = PoseStamped()
    pose.pose.position.x = final_tf[0,-1]
    pose.pose.position.y = final_tf[1,-1]
    pose.pose.position.z = final_tf[2,-1]
    quat = mat2quat(final_tf[0:3,0:3])
    pose.pose.orientation.x = quat[1]
    pose.pose.orientation.y = quat[2]
    pose.pose.orientation.z = quat[3]
    pose.pose.orientation.w = quat[0]
    pose.header.frame_id = 'base'
    pose.header.stamp.secs = rospy.get_rostime().secs
    pose.header.stamp.nsecs = rospy.get_rostime().nsecs
    publisher.publish(pose)
    
def tfcallback_handright(msg):
    publish_pose(msg, pose_pub_handright)
    
def tfcallback_handleft(msg):
    publish_pose(msg, pose_pub_handleft)
    
def tfcallback_headband(msg):
    publish_pose(msg, pose_pub_headband)

def tfcallback_ee(msg):
    publish_pose(msg, pose_pub_ee)
    
if __name__ == "__main__":
    rospy.init_node("transform_optitrack")
    global pose_pub_handright
    global pose_pub_handleft
    global pose_pub_headband
    global pose_pub_ee
    pose_pub_handright = rospy.Publisher("/vrpn_client_node/HandRight_tf/pose", PoseStamped, queue_size=10)
    pose_pub_handleft = rospy.Publisher("/vrpn_client_node/HandLeft_tf/pose", PoseStamped, queue_size=10)
    pose_pub_headband = rospy.Publisher("/vrpn_client_node/HeadBand_tf/pose", PoseStamped, queue_size=10)
    pose_pub_ee = rospy.Publisher("/vrpn_client_node/EndEffector_tf/pose", PoseStamped, queue_size=10)
    rospy.Subscriber("/vrpn_client_node/HandRight/pose", PoseStamped, tfcallback_handright)
    rospy.Subscriber("/vrpn_client_node/HandLeft/pose", PoseStamped, tfcallback_handleft)
    rospy.Subscriber("/vrpn_client_node/HeadBand/pose", PoseStamped, tfcallback_headband)
    rospy.Subscriber("/vrpn_client_node/EndEffector/pose", PoseStamped, tfcallback_ee)
    rospy.spin()