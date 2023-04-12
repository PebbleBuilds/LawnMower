#!/usr/bin/env python

# ==============================================================================
# === Imports =================================================================
# ==============================================================================
import copy
import math
from threading import Lock

import cv2
import numpy as np
import rospy
import tf
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image

from constants import *

# ==============================================================================
# === Global Variables =========================================================
# ==============================================================================
Q_MATRIX = None
DISPARITY_MAP = None
OBSTACLE_POINTS = None
DRONE_POSE = None

CAM_INFO1 = None
CAM_INFO2 = None

MAPX1 = None
MAPY1 = None
MAPX2 = None
MAPY2 = None

OBSTACLE_RADIUS = 0.1524 # meters

BRIDGE = CvBridge()

# ==============================================================================
# === Helper Functions =========================================================
# ==============================================================================
def update_disparity_map(msg):
    global DISPARITY_MAP
    DISPARITY_MAP = msg

def update_drone_pose(msg):
    global DRONE_POSE
    DRONE_POSE = msg

def camera_info_cb1(msg):
    global CAM_INFO1
    CAM_INFO1 = copy.deepcopy(msg)

def camera_info_cb2(msg):
    global CAM_INFO2
    CAM_INFO2 = copy.deepcopy(msg)

def init_maps():
    """
        Use to obtain the Q matrix for the stereo camera setup
    """
    global MAPX1, MAPY1, MAPX2, MAPY2

    K1 = np.array(CAM_INFO1.K).reshape(3,3)
    D1 = np.array(CAM_INFO1.D)
    K2 = np.array(CAM_INFO2.K).reshape(3,3)
    D2 = np.array(CAM_INFO2.D)
    T = np.array([BASELINE, 0, 0]) # 64 mm baseline
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, IMG_SIZE_WH, R=np.eye(3), T=T
    )

    MAPX1, MAPY1 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, R1, P1, size=IMG_SIZE_WH, m1type=cv2.CV_32FC1
    )
    MAPX2, MAPY2 = cv2.fisheye.initUndistortRectifyMap(
        K2, D2, R2, P2, size=IMG_SIZE_WH, m1type=cv2.CV_32FC1
    )

    global Q_MATRIX
    Q_MATRIX = Q


def process_point_cloud(disparity, Q_MATRIX):
    """
    """
    
    # Step 1: Reproject disparity map to 3D point cloud
    point_cloud = cv2.reprojectImageTo3D(disparity, Q_MATRIX)

    # Step 2: Filter out points with depth value less than and greater than thresholds
    min_threshold = 0.3 # meters
    max_threshold = 2.5 # meters

    mask = np.logical_and(point_cloud[:,:,2] > min_threshold, point_cloud[:,:,2] < max_threshold)
    point_cloud = point_cloud[mask]

    # Step 3: Find all points at y = 0
    mask = point_cloud[:,:,1] == 0
    point_cloud = point_cloud[mask]

    # Step 4: Find the closest point to the drone
    min_distance = np.min(point_cloud[:,:,2])
    mask = point_cloud[:,:,2] == min_distance
    closest_obstacles = point_cloud[mask]

    # Step 5: Use the coordinate of the closest obstacle to locate obstacle in map frame
    located_obstacle_pose_stamp = locate_obstacles(closest_obstacles)

    # Step 6: Transform the located obstacle to world frame
    world_obstacle_pose_stamp = transform_map_to_world(located_obstacle_pose_stamp)

    return world_obstacle_pose_stamp

def locate_obstacles(closest_obstacles):
    """
    The drone's position relative to its starting location, lets call this map frame, is kept in a PoseStamped() message called DRONE_POSE. 
    We also know the quaternion orientation of the drone from that message. The stereo camera of the drone points forward and captures a disparity map, 
    which is converted into a 3D point cloud. Using this point cloud, we want to find the closest obstacle point to the drone in the point cloud frame, 
    then convert that obstacle point to the map frame by performing some calculations. The return is a PoseStamped() message of the obstacle in map frame.
    """

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = rospy.Time.now()

    # Find the closest obstacle point to the principal point of point cloud frame
    closest_obstacle = closest_obstacles[np.argmin(np.abs(closest_obstacles[:,0]))]

    # Extract drone's current position and orientation in map frame
    drone_position = np.array([DRONE_POSE.pose.position.x, DRONE_POSE.pose.position.y, DRONE_POSE.pose.position.z])
    drone_orientation = np.array([DRONE_POSE.pose.orientation.x, DRONE_POSE.pose.orientation.y, DRONE_POSE.pose.orientation.z, DRONE_POSE.pose.orientation.w])

    # Convert drone's quaternion orientation to rotation matrix
    drone_rotation_matrix = tf.transformations.quaternion_matrix(drone_orientation)[:3, :3]

    # Transform the closest_obstacle coordinates from the point cloud frame to the map frame
    obstacle_position_map_frame = drone_position + drone_rotation_matrix.dot(closest_obstacle)

    # Create a PoseStamped message
    pose_stamped.pose.position = Point(*obstacle_position_map_frame)
    pose_stamped.pose.orientation = Quaternion(0, 0, 0, 1) # Set the orientation to the identity quaternion since we're only interested in position

    return pose_stamped


# TODO: Functions for transforming points and detecting obstacles
def transform_map_to_world(pose_array):
    # Transform points in PoseArray from map to world frame
    transformed_pose_array = PoseArray()
    transformed_pose_array.header.frame_id = "world"
    transformed_pose_array.header.stamp = rospy.Time.now()
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "world"
    for pose in pose_array.poses:
        transformed_pose = Pose()
        # TODO: Fill this in
        transformed_pose_array.poses.append(transformed_pose)
    return transformed_pose_array


#===============================================================================
# Main
#===============================================================================

def main():

    global UNDISTORT_PUB1, CAMERA_INFO_PUB1, UNDISTORT_PUB2, CAMERA_INFO_PUB2, DISPARITY_MAP, Q_MATRIX, BRIDGE, OBSTACLE_POINTS

    # Initialize node
    rospy.init_node("locate_closest_obstacle_node")

    # Subscribers
    rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, camera_info_cb1)
    rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, camera_info_cb2)
    rospy.Subscriber(DISPARITY_MAP_TOPIC, Image, update_disparity_map)

    # TODO: subscribe to drone's current position in world frame (use tf tree)
    rospy.Subscriber(MAVROS_POSE_TOPIC, PoseStamped, update_drone_pose)

    # Publishers
    OBSTACLE_POINTS = rospy.Publisher("world_obstacles", PoseArray, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if CAM_INFO1 is not None and CAM_INFO2 is not None and MAPX1 is None:
            init_maps()
        if DISPARITY_MAP is not None and Q_MATRIX is not None and DRONE_POSE is not None:
            # Process disparity map and publish obstacle points in world frame
            disparity = BRIDGE.imgmsg_to_cv2(DISPARITY_MAP, desired_encoding="passthrough")
            world_obstacle_pose_stamp = process_point_cloud(disparity, Q_MATRIX)
            OBSTACLE_POINTS.publish(world_obstacle_pose_stamp)
            
        rate.sleep()


if __name__ == "__main__":
    main()








#===============================================================================
# Old Code (for reference)
#===============================================================================
# def locate_obstacles(peak_points):
#     """
#     Steps:
#     1. Take the drone's current position in map frame
#     2. The peak point is the obstacle location in map frame
#     3. Using the peak point, find the obstacle location in map frame
#     Note: Obstacles are cylinders with radius 0.1524 m offset
#     from the peak point in the direction of the drone's current position
#     Warning: Ensure the x, y, z axes are aligned between the map and point cloud frames
#     peak_points are point cloud points in map frame:
#         X: The horizontal distance of the point from the camera's principal point (image center) along the image plane's X-axis. Positive X values are to the right of the principal point.
#         Y: The vertical distance of the point from the camera's principal point (image center) along the image plane's Y-axis. Positive Y values are below the principal point.
#         Z: The depth or distance of the point from the camera along the camera's optical (Z) axis.
    
#     In drone (map frame) coordinates, the x-axis points forward, the y-axis points to the left, and the z-axis points up.
#     Map to Cloud (x, y, z) = (z, -x, -y)
#     """
#     # Initialize poseArray()
#     pose_array = PoseArray()
#     pose_array.header.frame_id = "map"
#     pose_array.header.stamp = rospy.Time.now()
#     # For each peak_point, find the obstacle location in map frame
#     for peak_point in peak_points:   
#         # Extract drone's current position in map frame
#         drone_x = DRONE_POSE.pose.position.x
#         drone_y = DRONE_POSE.pose.position.y
#         drone_z = DRONE_POSE.pose.position.z
#         # For each peak_point, find the obstacle location in map frame
#         for peak_point in peak_points:
#             # Extract peak point's coordinates in map frame
#             peak_x = peak_point[0] # assume meters
#             peak_y = peak_point[1]
#             peak_z = peak_point[2]
#             # Find angle between peak point and drone's current position
#             theta = math.atan2(peak_z, peak_x)
#             theta_degrees = math.degrees(theta)
#             # Convert theta 
#             theta = math.radians(theta_degrees - 90)
#             # Find the obstacle location in map frame
#             # TODO: Check the angles
#             obstacle_x = drone_x + peak_z + 0.1524 * math.cos(theta)
#             obstacle_y = drone_y - peak_x - 0.1524 * math.sin(theta)
#             obstacle_z = drone_z - peak_y
#             # Add obstacle location in map frame to poseArray()
#             pose = Pose()
#             pose.position.x = obstacle_x
#             pose.position.y = obstacle_y
#             pose.position.z = obstacle_z
#             pose_array.poses.append(pose)
#     return pose_array