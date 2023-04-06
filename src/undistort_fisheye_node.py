#!/usr/bin/env python

import copy
import math
from threading import Lock

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import CameraInfo, Image

from constants import *

UNDISTORT_PUB1 = None
CAMERA_INFO_PUB1 = None
UNDISTORT_PUB2 = None
CAMERA_INFO_PUB2 = None

MAPX1 = None
MAPY1 = None
MAPX2 = None
MAPY2 = None

CAM_INFO1 = None
CAM_INFO2 = None

BRIDGE = CvBridge()

Q_MATRIX = None
DISPARITY_MAP = None
OBSTACLE_POINTS = None
DRONE_POSE = None

def img_cb1(msg):
    img_cb(msg, UNDISTORT_PUB1, MAPX1, MAPY1)

def img_cb2(msg):
    img_cb(msg, UNDISTORT_PUB2, MAPX2, MAPY2)

def img_cb(msg, undistort_pub, mapx, mapy):
    if mapx is None or mapy is None or undistort_pub is None:
        return
    img_distorted = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    img_undistorted = cv2.remap(
        img_distorted,
        mapx,
        mapy,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    # crop top and bottom based on DOWNSCALE_H
    orig_height = img_undistorted.shape[0]
    new_height = orig_height//DOWNSCALE_H
    # take center of image of new height
    img_undistorted = img_undistorted[
        (orig_height - new_height)//2 : (orig_height + new_height)//2, :
    ]
    # convert from mono8 to bgr8
    img_undistorted = cv2.cvtColor(img_undistorted, cv2.COLOR_GRAY2BGR)
    output_msg = BRIDGE.cv2_to_imgmsg(img_undistorted, encoding="bgr8")
    output_msg.header = msg.header
    undistort_pub.publish(output_msg)

def camera_info_cb1(msg):
    global CAM_INFO1
    CAM_INFO1 = copy.deepcopy(msg)
    camera_info_cb(msg, CAMERA_INFO_PUB1)

def camera_info_cb2(msg):
    global CAM_INFO2
    CAM_INFO2 = copy.deepcopy(msg)
    camera_info_cb(msg, CAMERA_INFO_PUB2)

def camera_info_cb(msg, camera_info_pub):
    if camera_info_pub is None:
        return
    msg.distortion_model = "plumb_bob"
    msg.D = [0, 0, 0, 0, 0]
    # downscale K and P
    msg.K = list(msg.K)
    msg.K[5] = msg.K[5]/DOWNSCALE_H # optical center
    msg.P = list(msg.P)
    msg.P[6] = msg.P[6]/DOWNSCALE_H # optical center
    camera_info_pub.publish(msg)

def update_disparity_map(msg):
    global DISPARITY_MAP
    DISPARITY_MAP = msg

def update_drone_pose(msg):
    global DRONE_POSE
    DRONE_POSE = msg

def process_point_cloud(disparity, Q_MATRIX):
    
    # Step 1: Calculate the 3D point cloud from disparity map and Q matrix
    point_cloud = cv2.reprojectImageTo3D(disparity, Q_MATRIX)

    # Step 1.1: Extract depth values (Z coordinates) from the point cloud
    depth_values = point_cloud[:, :, 2]

    # Step 1.2: Apply the threshold to remove depth values below a certain value (e.g., 1 meter)
    depth_lower_threshold = 1
    depth_values_masked = np.ma.masked_less(depth_values, depth_lower_threshold)

    # Step 1.3: Create a histogram
    hist, bin_edges = np.histogram(depth_values_masked.compressed(), bins=50)

    # Step 1.4: Find the closest depth value with the highest frequency
    peak_depth_indices = np.argwhere(hist == np.amax(hist)).flatten()
    peak_depth_values = (bin_edges[peak_depth_indices] + bin_edges[peak_depth_indices + 1]) / 2
    closest_peak_depth_value = peak_depth_values[np.argmin(np.abs(peak_depth_values - depth_lower_threshold))]

    # Step 2: Find respective points in 3D frame that have depth equal to the closest peak value of the histogram
    depth_tolerance = 0.1  # Adjust this value based on the required precision
    depth_near_peak = np.isclose(depth_values, closest_peak_depth_value, atol=depth_tolerance)
    peak_points = point_cloud[depth_near_peak]

    # Only return peak_points at the height of the drone in map frame
    current_height = DRONE_POSE.pose.position.z
    height_tolerance = 0.1
    height_near_peak = np.isclose(peak_points[:, 2], current_height, atol=height_tolerance)
    peak_points = peak_points[height_near_peak]

    # Step 3: Obstacle detection using world_points
    # Assuming you have a function `detect_obstacles(points)` that performs obstacle detection
    # detected_obstacles = detect_obstacles(world_points)
    detected_obstacles = detect_obstacles(peak_points)

    # Step 4: Transform coordinates from map to world frame
    # Assuming you have a function `transform_map_to_world(points)` that performs this operation
    # world_points = transform_map_to_world(peak_points)
    world_points = transform_map_to_world(detected_obstacles)

    return world_points # PoseArray Type returned

def detect_obstacles(peak_points):
    """
    Steps:
    1. Take the drone's current position in map frame
    2. The peak point is the obstacle location in map frame
    3. Using the peak point, find the obstacle location in map frame

    Note: Obstacles are cylinders with radius 0.1524 m offset
    from the peak point in the direction of the drone's current position

    Warning: Ensure the x, y, z axes are aligned between the map and point cloud frames

    peak_points are point cloud points in map frame:
        X: The horizontal distance of the point from the camera's principal point (image center) along the image plane's X-axis. Positive X values are to the right of the principal point.
        Y: The vertical distance of the point from the camera's principal point (image center) along the image plane's Y-axis. Positive Y values are below the principal point.
        Z: The depth or distance of the point from the camera along the camera's optical (Z) axis.
    
    In drone (map frame) coordinates, the x-axis points forward, the y-axis points to the left, and the z-axis points up.
    Map to Cloud (x, y, z) = (z, -x, -y)
    """
    # Initialize poseArray()
    pose_array = PoseArray()
    pose_array.header.frame_id = "map"
    pose_array.header.stamp = rospy.Time.now()

    # Extract drone's current position in map frame
    drone_x = DRONE_POSE.pose.position.x
    drone_y = DRONE_POSE.pose.position.y
    drone_z = DRONE_POSE.pose.position.z

    # For each peak_point, find the obstacle location in map frame
    for peak_point in peak_points:
        # Extract peak point's coordinates in map frame
        peak_x = peak_point[0] # assume meters
        peak_y = peak_point[1]
        peak_z = peak_point[2]

        # Find angle between peak point and drone's current position
        theta = math.atan2(peak_z, peak_x)
        theta_degrees = math.degrees(theta)

        # Convert theta 
        theta = math.radians(theta_degrees - 90)

        # Find the obstacle location in map frame
        # TODO: Check the angles
        obstacle_x = drone_x + peak_z + 0.1524 * math.cos(theta)
        obstacle_y = drone_y - peak_x - 0.1524 * math.sin(theta)
        obstacle_z = drone_z - peak_y

        # Add obstacle location in map frame to poseArray()
        pose = Pose()
        pose.position.x = obstacle_x
        pose.position.y = obstacle_y
        pose.position.z = obstacle_z
        pose_array.poses.append(pose)

    return pose_array

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

def init_maps():
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

def main():
    global UNDISTORT_PUB1, CAMERA_INFO_PUB1, UNDISTORT_PUB2, CAMERA_INFO_PUB2, DISPARITY_MAP, Q_MATRIX, BRIDGE, OBSTACLE_POINTS
    rospy.init_node("undistort_img_node")
    # subscribers
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, img_cb1)
    rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, camera_info_cb1)
    rospy.Subscriber("/camera/fisheye2/image_raw", Image, img_cb2)
    rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, camera_info_cb2)
    rospy.Subscriber(DISPARITY_MAP_TOPIC, Image, update_disparity_map)
    # TODO: subscribe to drone's current position in world frame (use tf tree)
    rospy.Subscriber(MAVROS_POSE_TOPIC, PoseStamped, update_drone_pose)

    # publishers
    OBSTACLE_POINTS = rospy.Publisher("world_obstacles", PoseArray, queue_size=1)

    UNDISTORT_PUB1 = rospy.Publisher("undistorted1/image_raw", Image, queue_size=1)
    CAMERA_INFO_PUB1 = rospy.Publisher(
        "undistorted1/camera_info", CameraInfo, queue_size=1
    )
    UNDISTORT_PUB2 = rospy.Publisher("undistorted2/image_raw", Image, queue_size=1)
    CAMERA_INFO_PUB2 = rospy.Publisher(
        "undistorted2/camera_info", CameraInfo, queue_size=1
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if CAM_INFO1 is not None and CAM_INFO2 is not None and MAPX1 is None:
            init_maps()

        if DISPARITY_MAP is not None and Q_MATRIX is not None and DRONE_POSE is not None:
            # Process disparity map and publish obstacle points in world frame
            disparity = BRIDGE.imgmsg_to_cv2(DISPARITY_MAP, desired_encoding="passthrough")
            detected_obstacles = process_point_cloud(disparity, Q_MATRIX)
            OBSTACLE_POINTS.publish(detected_obstacles)
            
        rate.sleep()


if __name__ == "__main__":
    main()
