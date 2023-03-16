#!/usr/bin/env python

# TODO add documentation for all functions

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from utils.pose_utils import create_posestamped, np2posestamped, posestamped2np


class WaypointFollower:
    def __init__(
        self,
        radius: float = 0.5,
        hold_time: int = 2,
        launch_height: float = 1,
        waypoints=None,
    ) -> None:
        self.waypoint_idx: int = 0
        self.waypoints_received: bool = False
        if waypoints is not None:
            self.set_waypoints(waypoints)
        self.radius = radius
        self.hold_time = hold_time
        self.state = "Init"  # Launch, Test, Land, Abort, Init

        self.current_pose = create_posestamped([0, 0, 0])

        self.origin_setpoint = create_posestamped([0, 0, 0])

        self.setpoint = create_posestamped([0, 0, 0])

        self.launch_height = launch_height

    def set_waypoints(self, waypoints):
        if self.waypoints_received:
            return
        print("Setting waypoints:", waypoints)
        self.waypoints = waypoints
        self.waypoints_received = True

    def set_state(self, state: str):
        assert state in ["Launch", "Test", "Land", "Abort", "Init"], (
            "Invalid state " + state
        )
        self.state = state

    def update_pose(self, drone_pose: PoseStamped):
        self.current_pose = drone_pose

    def get_setpoint(self, drone_pose: PoseStamped) -> PoseStamped:
        print("State: " + self.state)
        if self.state == "Launch":
            # set setpoint to point above origin
            self.setpoint = self.origin_setpoint
            self.setpoint.pose.position.z = self.launch_height
        elif self.state == "Test":
            if not self.waypoints_received:
                print("Waiting for waypoints")
                return self.setpoint
            return self.handle_test(drone_pose)
        elif self.state == "Land":
            # set setpoint to origin
            self.setpoint = self.origin_setpoint
        elif self.state == "Abort":
            self.setpoint = self.origin_setpoint
            self.setpoint.pose.position.x = drone_pose.pose.position.x
            self.setpoint.pose.position.y = drone_pose.pose.position.y
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

    def handle_test(self, drone_pose):
        # check distance to current waypoint
        dist = np.linalg.norm(
            np2posestamped(drone_pose.pose.position)
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
            # if waypoint index exceeds number of waypoints, reset to 0
            if self.waypoint_idx >= self.waypoints.shape[0]:
                self.waypoint_idx = 0
                print("Waypoints complete. Resetting to first waypoint.")
            # reset timer
            self.hold_timer = 0
            self.start_time = None
        self.setpoint = create_posestamped(self.waypoints[self.waypoint_idx, :])
        return self.setpoint
