#!/usr/bin/env python

# TODO add documentation for all functions

import numpy as np
import rospy
import tf

from pose_utils import create_posestamped, posestamped2np, pose2np


class WaypointFollower:
    def __init__(
        self,
        radius=0.05,
        hold_time=2,
        launch_height=1,
        waypoints=None,
    ):
        self.global_waypoint_idx = 0
        self.waypoints_received = False
        if waypoints is not None:
            self.set_waypoints(waypoints)
        self.radius = radius
        self.hold_time = hold_time
        self.state = "Init"  # Launch, Test, Land, Abort, Init

        self.current_pose = create_posestamped([0, 0, 0])

        self.origin_setpoint = create_posestamped([0, 0, 0])

        self.setpoint = create_posestamped([0, 0, 0])
        self.launch_height = launch_height
        self.test_initialized = False

        self.listener = tf.TransformListener()
        # initialize timers and waypoint index
        self.global_waypoint_idx = 0
        self.start_time = None
        self.test_initialized = True

    def set_waypoints(self, waypoints):

        if self.waypoints_received:
            return
        print("Setting waypoints:", waypoints)
        # convert from vicon_world frame to vicon_inertial frame
        waypoints_aug = np.matmul(self.H_vi, waypoints_aug.T)  # [4, num_waypoints]
        waypoints_aug = waypoints_aug.T  # [num_waypoints, 4]

        self.waypoints = waypoints_aug[:, :3]
        print("Transformed wapoints: ", self.waypoints)
        self.waypoints_received = True

    def set_state(self, state):
        assert state in ["Launch", "Test", "Land", "Abort", "Init"], (
            "Invalid state " + state
        )
        self.state = state

    def update_pose(self, drone_pose):
        self.current_pose = drone_pose

    def get_setpoint(self):
        # print("State: " + self.state)
        if self.state == "Launch":
            # set setpoint to point above origin
            self.setpoint = self.origin_setpoint
            self.setpoint.pose.position.z = self.launch_height
        elif self.state == "Test":
            if not self.waypoints_received:
                print("Waiting for waypoints")
                return self.setpoint
            return self.handle_test()
        elif self.state == "Land":
            # set setpoint to origin
            self.setpoint = self.origin_setpoint
        elif self.state == "Abort":
            self.setpoint = self.origin_setpoint
            self.setpoint.pose.position.x = self.current_pose.pose.position.x
            self.setpoint.pose.position.y = self.current_pose.pose.position.y
        elif self.state == "Init":
            self.setpoint = self.origin_setpoint
        else:
            print("Invalid state. Landing drone.")
            self.state = "Land"
            self.setpoint = self.origin_setpoint
        return self.setpoint

    def handle_test(self):
        # check distance to current waypoint
        dist = np.linalg.norm(
            posestamped2np(self.current_pose)
            - self.waypoints[self.global_waypoint_idx, :]
        )
        # if within radius, increment timer
        if dist < self.radius and self.start_time is None:
            print("starting timer on waypoint", self.global_waypoint_idx)
            self.start_time = rospy.get_time()
            print(rospy.get_time())
        # if timer exceeds hold_time, increment waypoint
        if (
            self.start_time is not None
            and rospy.get_time() - self.start_time > self.hold_time
        ):
            self.global_waypoint_idx += 1
            print("waypoint reached, indexing to next waypoint")
            # if waypoint index exceeds number of waypoints, reset to 0
            if self.global_waypoint_idx >= self.waypoints.shape[0]:
                self.global_waypoint_idx = 0
                print("Waypoints complete. Resetting to first waypoint.")
            # reset timer
            self.start_time = None
        self.setpoint = create_posestamped(self.waypoints[self.global_waypoint_idx, :])
        return self.setpoint
