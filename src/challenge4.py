#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse

from constants import *
from pose_utils import create_posestamped, pose2np
from waypoint_follower import WaypointFollower

WF = None

# Service callbacks
def callback_launch(request):
    rospy.loginfo("Launch Requested.")
    WF.set_state("Launch")
    return EmptyResponse()


def callback_test(request):
    rospy.loginfo("Test Requested.")
    WF.set_state("Test")
    return EmptyResponse()


def callback_land(request):
    rospy.loginfo("Land Requested.")
    WF.set_state("Land")
    return EmptyResponse()


def callback_abort(request):
    rospy.loginfo("Abort Requested.")
    WF.set_state("Abort")
    return EmptyResponse()


def callback_pose(msg):
    WF.update_pose(msg)


def callback_vicon(msg):
    WF.set_vicon_tf(msg.transform)


def callback_waypoints(msg):
    WF.set_waypoints(msg.poses)


# Main node
def comm_node():
    global WF
    # Do not change the node name and service topics!
    rospy.init_node(NAME)
    # TODO change params to get from launch file
    WF = WaypointFollower(
        radius=0.5,
        hold_time=1,
        launch_height=1,
        waypoints=None,
        dance=True,
        dance_size=0.35,
    )
    # services
    srv_launch = rospy.Service(LAUNCH_TOPIC, Empty, callback_launch)
    srv_test = rospy.Service(TEST_TOPIC, Empty, callback_test)
    srv_land = rospy.Service(LAND_TOPIC, Empty, callback_land)
    srv_abort = rospy.Service(ABORT_TOPIC, Empty, callback_abort)
    # subscribers
    sub_waypoints = rospy.Subscriber(WAYPOINTS_TOPIC, PoseArray, callback_waypoints)
    rospy.Subscriber(VICON_POSE_TOPIC, TransformStamped, callback_vicon)
    rospy.Subscriber(MAVROS_POSE_TOPIC, PoseStamped, callback_pose)
    # publishers
    sp_pub = rospy.Publisher(
        MAVROS_SETPOINT_TOPIC, PoseStamped, queue_size=1
    )
    rospy.loginfo("Services, subscribers, publishers initialized")

    while not rospy.is_shutdown():
        setpoint = WF.get_setpoint()
        sp_pub.publish(setpoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    comm_node()
