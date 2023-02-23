
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray







if __name__ == '__main__':
    
    rospy.init_node("test")
    cmd_pub = rospy.Publisher("/arm_controller/command",JointTrajectory,queue_size=100)
    
    group_pub = rospy.Publisher("/joint_group_position_controller/command",Float64MultiArray,queue_size=100)
    while not rospy.is_shutdown():
        try:
            msg = JointTrajectory()
            msg.header.frame_id = "ur5"
            msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            
            # msg.points = JointTrajectoryPoint()
            positions = [0.0, -0., 0.0, 0.0, 0.0, 0.0]
            # velocities = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
            velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            accelerations = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
            effort = [0, 0, 0, 0, 0, 0]

            # for i in range(len(positions)):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            point.effort = effort
            point.time_from_start = rospy.Time.now()
            msg.points.append(point)

            print(msg)
            cmd_pub.publish(msg)
            
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            print("shutdown")