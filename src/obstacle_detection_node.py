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

    # Convert PointCloud2 message to numpy array
    pc = ros_numpy.numpify(msg)
    POINT_CLOUD = np.hstack([pc['x'], pc['z']]) # (x, z)
    PC_HEADER = msg.header
    plt.scatter(POINT_CLOUD[:,0], POINT_CLOUD[:,1])


def process_point_cloud():
    """
    This function takes the point cloud and finds the closest obstacle to the drone.
    It returns a PoseStamped() message of the obstacle in the world frame.
    """
    # Step 1: Reproject disparity map to 3D point cloud
    point_cloud = POINT_CLOUD.copy()

    # Step 3: Filter out points with depth value less than and greater than thresholds
    mask = np.logical_and(point_cloud[:,1] > MIN_THRESHOLD, point_cloud[:,1] < MAX_THRESHOLD)
    point_cloud = point_cloud[mask]

    # Step 4: Find the k closest obstacles to the drone using euclidean distance from the principal point
    # Assume principal point is at (0,0,0) in point cloud frame
    distances = np.linalg.norm(point_cloud, axis=1)
    closest_k_obstacles_inds = np.argsort(distances)[:K]
    # compute the median distance of the closest k obstacles
    median_x = np.median(point_cloud[closest_k_obstacles_inds, 0])
    median_z = np.median(point_cloud[closest_k_obstacles_inds, 1])
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
    obstacle_pose = PoseStamped()
    obstacle_pose.header = PC_HEADER
    # Update obstacle_pose with closest_obstacle coordinates
    obstacle_pose.point.x = closest_obstacle[0]
    obstacle_pose.point.y = 0
    obstacle_pose.point.z = closest_obstacle[1]

    # Transform the obstacle_pose coordinates from the point cloud frame to the world frame
    tf_cloud_to_world = TF_BUFFER.lookup_transform(VICON_DUMMY_FRAME_ID, PC_HEADER.frame_id, PC_HEADER.stamp, rospy.Duration(1.0))
    obstacle_pose.pose = tf2_geometry_msgs.do_transform_point(obstacle_pose.point, tf_cloud_to_world)

    return obstacle_pose


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
    OBSTACLE_POINTS = rospy.Publisher(OBSTACLE_DETECTION_OUTPUT, PointStamped, queue_size=1)
    rate = rospy.Rate(10)
    rospy.loginfo("initialized obstacle detection node")
    while not rospy.is_shutdown():
        if POINT_CLOUD is not None:
            # Process disparity map and publish obstacle points in world frame
            world_obstacle_pose_stamp = process_point_cloud()
            print(world_obstacle_pose_stamp)
            OBSTACLE_POINTS.publish(world_obstacle_pose_stamp)
        rate.sleep()

if __name__ == "__main__":
    main()
