#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from waypoint_follower import WaypointFollower
from pose_utils import create_posestamped, posestamped2np

TEST_SETPOINTS = np.array(
    [
        [0, 0, 1],
        [0, 1, 1],
        [1, 1, 1],
        [1, 0, 1],
    ]
)

WF = WaypointFollower(
    radius=0.5, hold_time=2, launch_height=1, waypoints=TEST_SETPOINTS
)


def callback_pose(msg):
    WF.update_pose(msg)


# Main node
def comm_node():
    # Do not change the node name and service topics!
    name = "rob498_drone_02"  # Change 00 to your team ID
    rospy.init_node(name)
    # subscribers
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback_pose)
    # publishers
    sp_pub = rospy.Publisher(
        "/mavros/setpoint_position/local", PoseStamped, queue_size=1
    )
    print("Services, subscribers, publishers initialized")
    WF.set_state("Test")
    while not rospy.is_shutdown():
        setpoint = WF.get_setpoint()
        sp_pub.publish(setpoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    comm_node()
