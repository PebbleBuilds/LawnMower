#!/usr/bin/env python

# ==============================================================================
# === Imports =================================================================
# ==============================================================================

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points

from constants import *

# ==============================================================================
# === Global Variables =========================================================
# ==============================================================================
POINT_CLOUD = None
OBSTACLE_POINTS = None

TF_BUFFER = tf2_ros.Buffer()
TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
BRIDGE = CvBridge()

# ==============================================================================
# === Helper Functions =========================================================
# ==============================================================================

def update_point_cloud(msg):
    """
    Converts the PointCloud2 message into a numpy array of points
    """
    global POINT_CLOUD

    # Convert PointCloud2 message to numpy array
    point_generator = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_list = list(point_generator)
    points_np = np.array(points_list, dtype=np.float32)
    POINT_CLOUD = points_np


def process_point_cloud():
    """
    This function takes the point cloud and finds the closest obstacle to the drone.
    It returns a PoseStamped() message of the obstacle in the world frame.
    """
    # Step 1: Reproject disparity map to 3D point cloud
    point_cloud = POINT_CLOUD

    # Step 2: Find all points at y = 0
    mask = point_cloud[:,:,1] == 0
    point_cloud = point_cloud[mask]

    # Step 3: Filter out points with depth value less than and greater than thresholds
    min_threshold = 0.3 # meters
    max_threshold = 2.5 # meters

    mask = np.logical_and(point_cloud[:,:,2] > min_threshold, point_cloud[:,:,2] < max_threshold)
    point_cloud = point_cloud[mask]

    # Step 4: Find the closest point to the drone using euclidean distance from the principal point
    # Assume principal point is at (0,0,0) in point cloud frame
    principal_point = np.array([0,0,0])
    distances = np.linalg.norm(point_cloud - principal_point, axis=1)
    closest_obstacle = point_cloud[np.argmin(distances)] # closest_obstacles is a 3D point in point cloud frame

    # Step 5: Locate obstacle in world frame
    located_obstacle_pose = locate_obstacles_in_world(closest_obstacle)
    
    return located_obstacle_pose

def locate_obstacles_in_world(closest_obstacle):
    """
    Converts obstacle pose from point cloud frame directly to world frame. 
    Drone frame is unnecessary here. 
    """
    # End frame is world frame
    obstacle_pose = PoseStamped()
    obstacle_pose.header.frame_id = CLOUD_FRAME_ID
    obstacle_pose.header.stamp = rospy.Time.now()
    # Update obstacle_pose with closest_obstacle coordinates
    obstacle_pose.pose.position = Point(*closest_obstacle)
    # Set the orientation to the identity quaternion since we're only interested in position
    obstacle_pose.pose.orientation = Quaternion(0, 0, 0, 1) 

    # Transform the obstacle_pose coordinates from the point cloud frame to the world frame
    tf_cloud_to_world = TF_BUFFER.lookup_transform(VICON_DUMMY_FRAME_ID, CLOUD_FRAME_ID, rospy.Time(), rospy.Duration(1.0))
    obstacle_pose.pose = tf2_geometry_msgs.do_transform_pose(obstacle_pose.pose, tf_cloud_to_world)

    return obstacle_pose


#===============================================================================
# Main
#===============================================================================

def main():

    global BRIDGE, OBSTACLE_POINTS

    # Initialize node
    rospy.init_node("locate_closest_obstacle_node")

    # Subscribers
    rospy.Subscriber(POINT_CLOUD_TOPIC, PointCloud2, update_point_cloud)

    # Publishers
    OBSTACLE_POINTS = rospy.Publisher(CLOSEST_OBSTACLE_TOPIC, PoseStamped, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if POINT_CLOUD is not None:
            # Process disparity map and publish obstacle points in world frame
            world_obstacle_pose_stamp = process_point_cloud()
            OBSTACLE_POINTS.publish(world_obstacle_pose_stamp)
            
        rate.sleep()


if __name__ == "__main__":
    main()
