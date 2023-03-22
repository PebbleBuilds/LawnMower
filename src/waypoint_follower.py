#!/usr/bin/env python

# TODO add documentation for all functions

import numpy as np
import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PoseStamped, TransformPoseStamped
from pose_utils import create_posestamped, np2posestamped, posestamped2np, vicontf_to_Hvi


class WaypointFollower:
    def __init__(
        self,
        radius = 0.5,
        hold_time = 2,
        launch_height= 1,
        waypoints=None,
    ):
        self.waypoint_idx = 0
        self.waypoints_received = False
        if waypoints is not None:
            self.set_waypoints(waypoints)
        self.radius = radius
        self.hold_time = hold_time
        self.state = "Init"  # Launch, Test, Land, Abort, Init

        self.current_pose = create_posestamped([0, 0, 0])

        self.origin_setpoint = create_posestamped([0, 0, 0])

        self.setpoint = create_posestamped([0, 0, 0])
        self.vicon_tf = TransformPoseStamped()
        self.launch_height = launch_height
        self.test_initialized = False

    def set_vicon_tf(self, vicon_tf):
        if self.vicon_tf is not None:
            self.vicon_tf = vicon_tf
            self.H_vi = vicontf_to_Hvi(vicon_tf)
            print("assigning vicon tf to", vicon_tf)

    def set_waypoints(self, waypoints):
        assert self.vicon is not None, "vicon tf not received, cannot set waypoints"

        if self.waypoints_received:
            return
        print("Setting waypoints:", waypoints)
        """
        C_vi = tf2_geometry_msgs.transform_to_matrix(self.vicon_tf.rotation)
        t_vi_i = np.array([
            self.vicon_tf.translation.x,
            self.vicon_tf.translation.y,
            self.vicon_tf.translation.z
        ])
        t_iv_v = -C_vi @ t_vi_i
        """
        waypoints_aug = np.hstack(waypoints, np.ones(waypoints.shape[0])) # [num_waypoints, 4]
        waypoints_aug = self.H_vi @ waypoints_aug.T # [4, num_waypoints]
        waypoints_aug = waypoints_aug.T # [num_waypoints, 4]

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
        print("State: " + self.state)
        if self.state == "Launch":
            # set setpoint to point above origin
            self.setpoint = self.origin_setpoint
            self.setpoint.pose.position.z = self.launch_height
        elif self.state == "Test":
            if not self.test_initialized:
                self.init_test()
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

    def init_test(self):
        # initialize timers and waypoint index
        self.waypoint_idx = 0
        self.hold_timer = 0
        self.start_time = None
        self.test_initialized = True

    def handle_test(self, ):
        # check distance to current waypoint
        dist = np.linalg.norm(
            posestamped2np(self.current_pose)
            - self.waypoints[self.waypoint_idx, :]
        )
        # if within radius, increment timer
        if dist < self.radius:
            if self.start_time is None:
                self.start_time = rospy.get_time()
            self.hold_timer += rospy.get_time() - self.start_time
        # if timer exceeds hold_time, increment waypoint
        if self.hold_timer > self.hold_time:
            self.waypoint_idx += 1
            print("waypoint reached, indexing to next waypoint")
            # if waypoint index exceeds number of waypoints, reset to 0
            if self.waypoint_idx >= self.waypoints.shape[0]:
                self.waypoint_idx = 0
                print("Waypoints complete. Resetting to first waypoint.")
            # reset timer
            self.hold_timer = 0
            self.start_time = None
        self.setpoint = create_posestamped(self.waypoints[self.waypoint_idx, :])
        return self.setpoint
