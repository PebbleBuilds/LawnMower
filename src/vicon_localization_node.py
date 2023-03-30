#!/usr/bin/env python

"""
Reads in vicon pose estimate, transforms to correct frame, publish to 
"""
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped

POSE = PoseStamped()


def callback_vicon(msg):
    global POSE
    # convert transform stamped to PoseStamped
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose.position.x = msg.transform.translation.x
    pose.pose.position.y = msg.transform.translation.y
    pose.pose.position.z = msg.transform.translation.z
    pose.pose.orientation = msg.transform.rotation
    POSE = pose


if __name__ == "__main__":
    group_name = rospy.get_param("~group_name", "rob498_drone_02")
    rospy.init_node("vicon2pixhawk_node")
    # subscribers
    vicon_topic = rospy.get_param("~vicon_topic", "/vicon/ROB498_Drone/ROB498_Drone")
    rospy.Subscriber(vicon_topic, TransformStamped, callback_vicon)
    # publishers
    mavros_topic = rospy.get_param("~mavros_topic", "/mavros/vision_pose/pose")
    vicon_pose_pub = rospy.Publisher("mavros_topic", PoseStamped, queue_size=1)

    print("vicon2pixhawk_node initialized")
    while not rospy.is_shutdown():
        vicon_pose_pub.publish(POSE)
