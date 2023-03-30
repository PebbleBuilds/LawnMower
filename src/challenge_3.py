#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse
from waypoint_follower import WaypointFollower
from pose_utils import create_posestamped, pose2np

WF = WaypointFollower(radius=0.5, hold_time=1, launch_height=1, waypoints=None, dance=True, dance_size=0.35)


# Service callbacks
def callback_launch(request):
    print("Launch Requested.")
    WF.set_state("Launch")
    return EmptyResponse()


def callback_test(request):
    print("Test Requested.")
    WF.set_state("Test")
    return EmptyResponse()


def callback_land(request):
    print("Land Requested.")
    WF.set_state("Land")
    return EmptyResponse()


def callback_abort(request):
    print("Abort Requested.")
    WF.set_state("Abort")
    return EmptyResponse()


def callback_pose(msg):
    WF.update_pose(msg)

def callback_vicon(msg):
    WF.set_vicon_tf(msg.transform)

def callback_waypoints(msg):
    # convert waypoints to numpy array
    WAYPOINTS = np.empty((len(msg.poses), 3))
    for i, pose in enumerate(msg.poses):
        WAYPOINTS[i] = pose2np(pose)
    # TODO compensate for offset between vicon markers and pixhawk center
    WF.set_waypoints(WAYPOINTS)


# Main node
def comm_node():
    # Do not change the node name and service topics!
    name = "rob498_drone_02"  # Change 00 to your team ID
    rospy.init_node(name)
    # services
    srv_launch = rospy.Service(name + "/comm/launch", Empty, callback_launch)
    srv_test = rospy.Service(name + "/comm/test", Empty, callback_test)
    srv_land = rospy.Service(name + "/comm/land", Empty, callback_land)
    srv_abort = rospy.Service(name + "/comm/abort", Empty, callback_abort)
    # subscribers
    sub_waypoints = rospy.Subscriber(
        "/" + name + "/comm/waypoints", PoseArray, callback_waypoints
    )
    rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback_vicon)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback_pose)
    # publishers
    sp_pub = rospy.Publisher(
        "/mavros/setpoint_position/local", PoseStamped, queue_size=1
    )
    
    print("Services, subscribers, publishers initialized")

    while not rospy.is_shutdown():
        setpoint = WF.get_setpoint()
        sp_pub.publish(setpoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    comm_node()
