#!/usr/bin/env python
# license removed for brevity
import rospy
import pose_tools
from geometry_msgs.msg import PoseStamped, Pose

stored_waypoint = PoseStamped()
stored_pose_estimate = PoseStamped()

def waypoint_callback(waypoint):
    stored_waypoint = waypoint

def pose_estimate_callback(pose_estimate):
    stored_pose_estimate = pose_estimate

def talker():
    pub = rospy.Publisher('~setpoint_position/local', PoseStamped, queue_size=10)
    rospy.Subscriber("lawnmower/waypoint", PoseStamped, waypoint_callback)
    rospy.Subscriber("lawnmower/pose_estimate", PoseStamped, pose_estimate_callback)
    rospy.init_node('pixhawk_commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if stored_pose_estimate.pose.position.x != 0.0:
            error = PoseStamped()
            error.header.seq = 1
            error.header.stamp = rospy.Time.now()
            error.header.frame_id = "pixhawk"

            T_2 = PoseStamped_2_mat(stored_waypoint)
            T_1 = PoseStamped_2_mat(stored_pose_estimate)
            error_T = T_inv(T_1) * T_2
            error.pose = Mat_2_posestamped(error_T)
            pub.publish(error)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass