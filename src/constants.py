#!/usr/bin/env python
import numpy as np
NAME = "rob498_drone_02"

# States
LAUNCH = 0
TEST = 1
LAND = 2
ABORT = 3

# frame_ids
VICON_ORIGIN_FRAME_ID = "vicon/world"
VICON_DUMMY_FRAME_ID = "vicon/dummy"
DRONE_FRAME_ID = "vicon/ROB498_Drone/ROB498_Drone"
BASE_LINK_FRAME_ID = "base_link"
LOCAL_ORIGIN_FRAME_ID = "map"
CLOUD_FRAME_ID = "camera_fisheye1_optical_frame"

# Service topics
LAUNCH_TOPIC = "/{}/comm/launch".format(NAME)
TEST_TOPIC = "/{}/comm/test".format(NAME)
LAND_TOPIC = "/{}/comm/land".format(NAME)
ABORT_TOPIC = "/{}/comm/abort".format(NAME)

# Subscriber topics
MAVROS_POSE_TOPIC = "/mavros/local_position/pose"
VICON_POSE_TOPIC = "/vicon/ROB498_Drone/ROB498_Drone"
WAYPOINTS_TOPIC = "/{}/comm/waypoints".format(NAME)
MAVROS_SETPOINT_TOPIC = "/mavros/setpoint_position/local"
PC_TOPIC = "/camera/points2"
POINT_CLOUD_TOPIC = "/camera/pointcloud2"
CLOSEST_OBSTACLE_TOPIC = "/locate_closest_obstacle_node"

# stereo camera constants
H, W = 800, 848
IMG_SIZE_WH = (W, H)
DOWNSCALE_H = 8
STEREO_SIZE_WH = (W, H//DOWNSCALE_H)
BASELINE = -18.2928466796875/286.1825866699219 # 64 mm baseline

DROP_FRAMES = 3

# obstacle parameters
OBSTACLE_RADIUS = 0.3 # meters
OBSTACLE_HEIGHT = 2.0 # meters
NUM_OBSTACLES = 4
INITIAL_OBSTACLE_POSITIONS = np.array([
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0]
])

#
K = 4
MIN_THRESHOLD = 0.3 # meters
MAX_THRESHOLD = 2.5 # meters

# PC clustering parameters
LEAF_SIZE = 0.1