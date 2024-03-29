#!/usr/bin/env python

# ==============================================================================
# === Imports =================================================================
# ==============================================================================

import numpy as np
import rospy
import ros_numpy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PointStamped 
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points

from constants import *
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ==============================================================================
# === Global Variables =========================================================
# ==============================================================================
POINT_CLOUD = None
PC_HEADER = None
OBSTACLE_POINTS = None
TF_BUFFER = None

# ==============================================================================
# === Helper Functions =========================================================
# ==============================================================================

plt.ion()

def update_point_cloud(msg):
    """
    Converts the PointCloud2 message into a numpy array of points
    """
    global POINT_CLOUD, PC_HEADER
    # Convert PointCloud2 message to numpy array using read_points
    POINT_CLOUD = np.array(list(read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))
    if POINT_CLOUD.shape[0] == 0 or len(POINT_CLOUD.shape)!=2 or POINT_CLOUD.shape[1]!=3:
        POINT_CLOUD=None
        return
    # filter NAN
    mask = np.logical_not(np.isnan(POINT_CLOUD).any(axis=1))
    POINT_CLOUD = POINT_CLOUD[mask]
    PC_HEADER = msg.header
    # plt.scatter(POINT_CLOUD[:,0], POINT_CLOUD[:,2])\
    plt.Circle((0,0), 5)


def process_point_cloud():
    """
    This function takes the point cloud and finds the closest obstacle to the drone.
    It returns a PoseStamped() message of the obstacle in the world frame.
    """
    # Step 1: Reproject disparity map to 3D point cloud
    point_cloud = POINT_CLOUD.copy()
    if PC_HEADER is None:
        return

    # Step 2: Filter out points with depth value less than and greater than thresholds
    try:
        mask = np.logical_and(point_cloud[:,2] > MIN_DIST, point_cloud[:,2] < MAX_DIST)
    except:
        return None
    point_cloud = point_cloud[mask]

    # Step 3: Find the k closest obstacles to the drone using euclidean distance from the principal point
    # Assume principal point is at (0,0,0) in point cloud frame
    distances = np.linalg.norm(point_cloud, axis=1)
    closest_k_obstacles_inds = np.argsort(distances)[:K]
    # compute the median distance of the closest k obstacles
    median_x = np.median(point_cloud[closest_k_obstacles_inds, 0])
    median_z = np.median(point_cloud[closest_k_obstacles_inds, 2])
    median_point = np.array([median_x, median_z])
    # Step 5: Locate obstacle in world frame
    located_obstacle_pose = locate_obstacle_in_world(median_point)
    
    return located_obstacle_pose

def locate_obstacle_in_world(closest_obstacle):
    """
    Converts obstacle pose from point cloud frame directly to world frame. 
    Drone frame is unnecessary here. 
    """
    # End frame is world frame
    obstacle_point = PointStamped()
    obstacle_point.header = PC_HEADER
    # Update obstacle_pose with closest_obstacle coordinates
    obstacle_point.point.x = closest_obstacle[0]
    obstacle_point.point.y = 0
    obstacle_point.point.z = closest_obstacle[1]

    # Transform the obstacle_pose coordinates from the point cloud frame to the world frame
    try:
        # tf_cloud_to_world = TF_BUFFER.lookup_transform(VICON_DUMMY_FRAME_ID, PC_HEADER.frame_id, rospy.Time(0))
        tf_cloud_to_world = TF_BUFFER.lookup_transform(VICON_DUMMY_FRAME_ID, PC_HEADER.frame_id, PC_HEADER.stamp)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("OD Failed to get transform from vicon")
        return None
    obstacle_point = tf2_geometry_msgs.do_transform_point(obstacle_point, tf_cloud_to_world)

    return obstacle_point


#===============================================================================
# Main
#===============================================================================

def main():

    global OBSTACLE_POINTS, TF_BUFFER

    # Initialize node
    rospy.init_node("locate_closest_obstacle_node")
    TF_BUFFER = tf2_ros.Buffer()
    TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)

    # Subscribers
    rospy.Subscriber(POINT_CLOUD_TOPIC, PointCloud2, update_point_cloud)

    # Publishers
    OBSTACLE_POINTS = rospy.Publisher(CLOSEST_OBSTACLE_TOPIC, PointStamped, queue_size=1)
    rate = rospy.Rate(10)
    rospy.loginfo("initialized obstacle detection node")
    while not rospy.is_shutdown():
        if POINT_CLOUD is not None:
            # Process disparity map and publish obstacle points in world frame
            world_obstacle_pose_stamp = process_point_cloud()
            if world_obstacle_pose_stamp is None:
                continue
            OBSTACLE_POINTS.publish(world_obstacle_pose_stamp)
        rate.sleep()

if __name__ == "__main__":
    main()
