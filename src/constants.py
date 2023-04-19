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
# VICON_DUMMY_FRAME_ID = "vicon/world"
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
OBSTACLE_POS_TOPIC = "dummy_obs_pose"
OBSTACLE_TYPE_TOPIC = "dummy_obs_type"

# Publisher topics
PLANNER_TOPIC = "next_waypoint"

PC_TOPIC = "/camera/points2"
POINT_CLOUD_TOPIC = "/camera/points2"
CLOSEST_OBSTACLE_TOPIC = "/locate_closest_obstacle_node"
TRACKER_OUTPUT = "/obstacles/tracked"

# stereo camera constants
H, W = 800, 848
IMG_SIZE_WH = (W, H)
DOWNSCALE_H = 8
STEREO_SIZE_WH = (W, H//DOWNSCALE_H)
BASELINE = -18.2928466796875/286.1825866699219 # 64 mm baseline
DROP_FRAMES = 3

# obstacle parameters
OBSTACLE_RADIUS = 1.0 # meters
OBSTACLE_HEIGHT = 2.0 # meters
FOS = 1
NUM_OBSTACLE_POINTS = 8
NUM_OBSTACLES = 4
INITIAL_OBSTACLE_POSITIONS = np.array([
    [1.5, 1.5],
    [1.5, -1.5],
    [-1.5, -1.5],
    [-1.5, 1.5]
])
# red is clockwise, true
INITIAL_OBSTACLE_CLOCKWISE = [True, True, False, False]

# arena boundary, anything outside detected should be ignored
X_BOUNDS_MIN = -6.0 # meters
X_BOUNDS_MAX = 6.0 # meters
Y_BOUNDS_MIN = -4.2 # meters
Y_BOUNDS_MAX = 4.2 # meters

#
K = 50
MIN_DIST = 0.5 # meters
MAX_DIST = 2.0 # meters
MAX_WIDTH = 1.0 # meters

# PC clustering parameters
LEAF_SIZE = 0.1
