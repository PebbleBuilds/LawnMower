#!/usr/bin/env python

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
OBSTACLE_POS_TOPIC = "dummy_obs_pose"
OBSTACLE_TYPE_TOPIC = "dummy_obs_type"

# Publisher topics
PLANNER = "next_waypoint"

# Obstacle constants
OBS_RADIUS = 0.2
FOS = 1/0.2

# stereo camera constants
H, W = 800, 848
IMG_SIZE_WH = (W, H)
DOWNSCALE_H = 8
STEREO_SIZE_WH = (W, H//DOWNSCALE_H)
BASELINE = -18.2928466796875/286.1825866699219 # 64 mm baseline