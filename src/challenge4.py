#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse

from constants import *
from waypoint_follower import WaypointFollower
from graph import DirectedGraph
from pose_utils import posestamped2np

WF = None

# Service callbacks
def callback_launch(request):
    rospy.loginfo("Launch Requested.")
    WF.set_state(LAUNCH)
    return EmptyResponse()


def callback_test(request):
    rospy.loginfo("Test Requested.")
    WF.set_state(TEST)
    return EmptyResponse()


def callback_land(request):
    rospy.loginfo("Land Requested.")
    WF.set_state(LAND)
    return EmptyResponse()


def callback_abort(request):
    rospy.loginfo("Abort Requested.")
    WF.set_state(ABORT)
    return EmptyResponse()


def callback_waypoints(msg):
    WF.set_waypoints(msg.poses)  
    graph.add_waypoints([posestamped2np(pose for pose in msg.poses)])

# Main node
def comm_node():
    global WF, graph
    # Do not change the node name and service topics!
    rospy.init_node(NAME)
    # TODO change params to get from launch file
    graph = DirectedGraph()
    WF = WaypointFollower(
        radius=0.5,
        hold_time=1,
        launch_height=1,
        waypoints=None,
        dance=True,
        dance_size=0.35,
    )
    # services
    rospy.Service(LAUNCH_TOPIC, Empty, callback_launch)
    rospy.Service(TEST_TOPIC, Empty, callback_test)
    rospy.Service(LAND_TOPIC, Empty, callback_land)
    rospy.Service(ABORT_TOPIC, Empty, callback_abort)
    # subscribers
    rospy.Subscriber(WAYPOINTS_TOPIC, PoseArray, callback_waypoints)
    # publishers
    sp_pub = rospy.Publisher(MAVROS_SETPOINT_TOPIC, PoseStamped, queue_size=1)
    rospy.loginfo("Services, subscribers, publishers initialized")

    while not rospy.is_shutdown():
        
        setpoint = WF.get_setpoint()
        if setpoint is not None:
            sp_pub.publish(setpoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    comm_node()
