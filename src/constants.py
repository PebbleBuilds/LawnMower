#!/usr/bin/env python

# States
LAUNCH = 0
TEST = 1
LAND = 2
ABORT = 3

VICON_ORIGIN_FRAME_ID = "/vicon/world"
VICON_DRONE_FRAME_ID = "/vicon/ROB498_Drone/ROB498_Drone"
DRONE_ORIGIN_FRAME_ID = "map"
DRONE_FRAME_ID= "local_position"

NAME = "rob498_drone_02"

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