#!/usr/bin/env python

# TODO add documentation for all functions

import numpy as np
import rospy
import tf2_ros

from pose_utils import create_posestamped, posestamped2np, tfstamped2posestamped, pose2np
from constants import *
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header


class WaypointFollower:
    def __init__(
        self,
        radius=0.1,
        hold_time=2,
        launch_height=1,
        waypoints=None,
        dance=False,
        dance_size=0.35,
    ):
        self.waypoints_received = False
        if waypoints is not None:
            self.set_waypoints(waypoints)
        self.radius = radius
        self.hold_time = hold_time
        self.state = LAND
        self.launch_height = launch_height
        # initialize setpoints
        self.origin_setpoint = create_posestamped(
            [0, 0, 0],
            orientation=[0, 0, -0.7071068, 0.7071068],  # checkerboard wall
            frame_id=VICON_DUMMY_FRAME_ID,
        )

        self.launch_setpoint = create_posestamped(
            [0, 0, launch_height],
            orientation=[0, 0, -0.7071068, 0.7071068],  # checkerboard wall
            frame_id=VICON_DUMMY_FRAME_ID,
        )

        # tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # initialize timers and waypoint index
        self.global_waypoint_idx = 0
        self.dance_waypoint_idx = 0
        self.waypoint_arrival_time = None

        self.last_setpoint_world = self.origin_setpoint


    def set_waypoints(self, waypoints):
        #if self.waypoints_received:
        #    return
        # print("Setting waypoints: \n", waypoints)
        # convert from vicon_world frame to vicon_inertial frame
        self.waypoints_world = waypoints
        self.waypoints_received = True

    def set_state(self, state):
        assert state in [LAUNCH, TEST, LAND, ABORT], "Invalid state " + state
        self.state = state

    def get_setpoint(self):
        if self.state == LAUNCH:
            # set setpoint to point above origin
            setpoint_world = self.origin_setpoint
            setpoint_world.pose.position.z = self.launch_height
        elif self.state == TEST:
            if not self.waypoints_received:
                rospy.loginfo("Waiting for waypoints")
                setpoint_world = self.last_setpoint_world
            else:
                setpoint_world = self.handle_test()
        elif self.state == LAND:
            # set setpoint to origin
            setpoint_world = self.origin_setpoint
        elif self.state == ABORT:
            setpoint_world = self.get_current_pose_world()
            if setpoint_world is None:
                setpoint_world = self.last_setpoint_world
            # bring drone down
            setpoint_world.pose.position.z = 0
        else:
            rospy.loginfo(self.state)
            rospy.loginfo("Invalid state. Landing drone.")
            self.state = "Land"
            setpoint_world = self.origin_setpoint
        # convert pose to local frame
        self.last_setpoint_world = setpoint_world
        setpoint_local = self.posestamped2local(setpoint_world)
        return setpoint_local

    def get_current_pose_world(self):
        try:
            t = self.tf_buffer.lookup_transform(
                VICON_DUMMY_FRAME_ID, DRONE_FRAME_ID, rospy.Time(0)
            )
            return tfstamped2posestamped(t)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn(
                "Waiting for transform from vicon to drone. Returning last setpoint."
            )
            return None

    def posestamped2local(self, pose_stamped):
        try:
            pose_local = self.tf_buffer.transform(pose_stamped, LOCAL_ORIGIN_FRAME_ID, rospy.Duration(1))
            return pose_local
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.loginfo(
                "Waiting for transform from vicon to local origin. Returning last setpoint."
            )
            return self.last_setpoint_world

    def handle_test(self):
        # TODO reimplement dancing
        setpoint = self.waypoints_world.poses[0]
        # convert from Pose to PoseStamped
        setpoint = PoseStamped(
            pose=setpoint, header=Header(frame_id=VICON_DUMMY_FRAME_ID)
        )
        return setpoint
        
