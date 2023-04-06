#!/usr/bin/env python

import rospy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import Pose, PoseArray
import copy
from constants import *

KALMAN_PUB = None
FILTER = KalmanFilter(dim_x=2 * NUM_OBSTACLES, dim_z=2 * NUM_OBSTACLES)
FILTER.x = INITIAL_OBSTACLE_POSITIONS.flatten()
FILTER.F = np.eye(2 * NUM_OBSTACLES)
FILTER.H = np.eye(2 * NUM_OBSTACLES)
# todo put in constants
FILTER.R = 0.1 * np.eye(2 * NUM_OBSTACLES)
FILTER.P = 1000 * np.eye(2 * NUM_OBSTACLES)
FILTER.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.01)
FILTERS = [copy.deepcopy(FILTER) for _ in range(NUM_OBSTACLES)]

def detections_cb(msg):
    # dissect pose array into a series of detections
    detections = []
    for pose in msg.poses:
        detections.append(np.array([pose.position.x, pose.position.y]))
    detections = np.vstack(detections)
    # perform data association with greedy algorithm between detections and objects
    # compute smallest distance between each detection and each object
    assigned_detections = np.zeros((NUM_OBSTACLES, 2))
    obstacle_positions = FILTER.x.reshape(NUM_OBSTACLES, 2)
    remaining_detections = detections.copy()
    for i, obstacle_position in enumerate(obstacle_positions):
        # compute distance between each detection and object
        distances = np.linalg.norm(remaining_detections - obstacle_position, axis=1)
        # find detection with smallest distance
        min_index = np.argmin(distances)
        # assign detection to object with smallest distance
        assigned_detections[i, :] = remaining_detections[min_index, :]  # (x, y)
        # remove detection from remaining detections
        remaining_detections = np.delete(remaining_detections, min_index, axis=0)
    # update kalman filter
    assigned_detections = assigned_detections.flatten()
    FILTER.update(assigned_detections)
    # convert kalman filter output to pose array
    tracked_poses = PoseArray()
    tracked_poses.header.stamp = rospy.Time.now()
    tracked_poses.header.frame_id = VICON_ORIGIN_FRAME_ID
    for i in range(NUM_OBSTACLES):
        pose = Pose()
        pose.position.x = FILTER.x[2 * i]
        pose.position.y = FILTER.x[2 * i + 1]
        tracked_poses.poses.append(pose)
    # publish kalman filter output
    KALMAN_PUB.publish(tracked_poses)


if __name__ == "__main__":
    rospy.init_node("kalman_filter_node")
    # subscribers
    rospy.Subscriber("/obstacles/detections", PoseArray, detections_cb)
    # publishers
    KALMAN_PUB = rospy.Publisher("/obstacles/tracked", PoseArray, queue_size=1)
    while not rospy.is_shutdown():
        pass
