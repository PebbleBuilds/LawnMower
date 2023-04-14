#!/usr/bin/env python

import rospy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import Pose, PoseArray, Point, PointStamped
import copy
from constants import *

KALMAN_PUB = None
_filter = KalmanFilter(dim_x=2, dim_z=2)
_filter.F = np.eye(2)
_filter.H = np.eye(2)
# todo put in constants
_filter.R = 0.1 * np.eye(2)
_filter.P = 1000 * np.eye(2)
_filter.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.01)
FILTERS = [copy.deepcopy(_filter) for _ in range(NUM_OBSTACLES)]
# assign corresponding initial state to each filter
for i, filter in enumerate(FILTERS):
    filter.x = INITIAL_OBSTACLE_POSITIONS[i]

def detections_cb(msg):
    global FILTERS
    if KALMAN_PUB is None:
        rospy.loginfo("Waiting for kalman filter publisher")
        return
    point = msg.point
    point_frame_id = msg.header.frame_id
    assert point_frame_id == VICON_DUMMY_FRAME_ID, "detections must be in vicon frame"
    # determine the quadrant of the point
    if point.x > 0 and point.y > 0:
        quadrant = 0
    elif point.x < 0 and point.y > 0:
        quadrant = 1
    elif point.x < 0 and point.y < 0:
        quadrant = 2
    else:
        quadrant = 3
    # update the corresponding kalman filter
    filter = FILTERS[quadrant]

    # update kalman filter
    filter.update(np.array([point.x, point.y]))

    # convert kalman filter output to pose array
    tracked_poses = PoseArray()
    tracked_poses.header.stamp = rospy.Time.now()
    tracked_poses.header.frame_id = VICON_DUMMY_FRAME_ID
    for i in range(NUM_OBSTACLES):
        pose = Pose()
        pose.position.x = filter.x[0]
        pose.position.y = filter.x[1]
        tracked_poses.poses.append(pose)

    # publish kalman filter output
    KALMAN_PUB.publish(tracked_poses)


if __name__ == "__main__":
    rospy.init_node("kalman_filter_node")
    # subscribers
    rospy.Subscriber(OBSTACLE_DETECTION_OUTPUT, PointStamped, detections_cb)
    # publishers
    KALMAN_PUB = rospy.Publisher(TRACKER_OUTPUT_TOPIC, PoseArray, queue_size=1)
    while not rospy.is_shutdown():
        pass
