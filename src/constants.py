#!/usr/bin/env python
import numpy as np
NAME = "rob498_drone_02"

# States
LAUNCH = 0
TEST = 1
LAND = 2
ABORT = 3

# frame_ids
VICON_ORIGIN_FRAME_ID = "/vicon/world"
DRONE_FRAME_ID = "/vicon/ROB498_Drone/ROB498_Drone"
LOCAL_ORIGIN_FRAME_ID = "map"

# Service topics
LAUNCH_TOPIC = "/{}/launch".format(NAME)
TEST_TOPIC = "/{}/test".format(NAME)
LAND_TOPIC = "/{}/land".format(NAME)
ABORT_TOPIC = "/{}/abort".format(NAME)

# Subscriber topics
MAVROS_POSE_TOPIC = "/mavros/local_position/pose"
VICON_POSE_TOPIC = "/vicon/ROB498_Drone/ROB498_Drone"
WAYPOINTS_TOPIC = "/{}/comm/waypoints".format(NAME)
MAVROS_SETPOINT_TOPIC = "/mavros/setpoint_position/local"

# stereo camera constants
H, W = 800, 848
IMG_SIZE_WH = (W, H)
DOWNSCALE_H = 8
STEREO_SIZE_WH = (W, H//DOWNSCALE_H)
BASELINE = -18.2928466796875/286.1825866699219 # 64 mm baseline

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