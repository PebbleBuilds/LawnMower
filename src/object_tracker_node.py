#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray, PointStamped
import copy
from constants import *

KALMAN_PUB = None
OBSTACLE_LOC = INITIAL_OBSTACLE_POSITIONS.copy()

def detections_cb(msg):
    print("detections_cb")
    global OBSTACLE_LOC
    detection = np.array([msg.point.x, msg.point.y])
    # DETERMINE QUADRANT
    if detection[0] > 0 and detection[1] > 0:
        quadrant = 0
    elif detection[0] > 0 and detection[1] < 0:
        quadrant = 1
    elif detection[0] < 0 and detection[1] < 0:
        quadrant = 2
    elif detection[0] > 0 and detection[1] < 0:
        quadrant = 3
    OBSTACLE_LOC[quadrant] = detection
    tracked_poses = PoseArray()
    tracked_poses.header.stamp = rospy.Time.now()
    tracked_poses.header.frame_id = VICON_DUMMY_FRAME_ID
    for i in range(NUM_OBSTACLES):
        pose = Pose()
        pose.position.x = OBSTACLE_LOC[i][0]
        pose.position.y = OBSTACLE_LOC[i][1]
        tracked_poses.poses.append(pose)
    # publish kalman filter output
    KALMAN_PUB.publish(tracked_poses)


if __name__ == "__main__":
    rospy.init_node("kalman_filter_node")
    # subscribers
    rospy.Subscriber(CLOSEST_OBSTACLE_TOPIC, PointStamped, detections_cb)
    # publishers
    KALMAN_PUB = rospy.Publisher("/obstacles/tracked", PoseArray, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()
