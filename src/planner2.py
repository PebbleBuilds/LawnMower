#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray, PointStamped
import tf2_ros
import copy
from constants import *
import numpy as np
from pose_utils import *
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs

WAYPOINTS_POSES = None # PoseArray
OBSTACLES = None # PoseArray

OBSTACLE_CLOCKWISE = [True, False, True, False] # red is clockwise green is anticlockwise

class CoolPlanner():
    def __init__(self):
        self.current_sp = Pose()
        self.current_sp.position.z = 1
        self.next_waypoint_idx = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sp_radius = 0.05 # meters
        self.yaw_tolerance = 0.1 # radians
        self.hold_time = 2
        self.drone_width = 0.2 # meters
        self.look_ahead = 1
        self.avoid_dist = 1 # metres
        self.start_time = None


    def orient_sp(self, sp):
        # sp is a posestamped
        # get the next waypoint
        next_waypoint = self.get_next_waypoint()
        # calculate the angle
        y = next_waypoint.y - sp.y
        x = next_waypoint.x - sp.x
        theta = np.arctan2(y,x)
        # create a new posestamped
        sp_new = copy.deepcopy(sp)
        sp_new.orientation = quaternion_from_euler(0,0,theta)
        return sp_new

    def posestamped_to_frame(self, posestamped, frame_id):
        try:
            posestamped_drone = self.tf_buffer.transform(posestamped, frame_id, rospy.Duration(1))
            return posestamped_drone
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.loginfo(
                "Waiting for transform from vicon to local origin. Returning given posestamped."
            )
            return self.posestamped

    def avoid_obstacle(self, obstacle_posestamped):
        # check which quadrant the obstacle is in world frame
        if obstacle_posestamped.pose.x > 0:
            if obstacle_posestamped.pose.y > 0:
                quadrant = 0
            else:
                quadrant = 1
        else:
            if obstacle_posestamped.pose.y > 0:
                quadrant = 3
            else:
                quadrant = 2
        left = OBSTACLE_CLOCKWISE[quadrant]
        # convert obstacle to drone frame
        obstacle_pose_d = self.posestamped_to_frame(obstacle_posestamped, DRONE_FRAME_ID)
        avoid_sp_d = copy.deepcopy(obstacle_pose_d)
        if left:
            avoid_sp_d.pose.y = avoid_sp_d.pose.y + self.avoid_dist
        else:
            avoid_sp_d.pose.y = avoid_sp_d.pose.y - self.avoid_dist
        return self.orient_sp(self.posestamped_to_frame(avoid_sp_d, VICON_DUMMY_FRAME_ID))

    def get_current_pose(self):
        try:
            current_pose = self.tf_buffer.lookup_transform(VICON_DUMMY_FRAME_ID, DRONE_FRAME_ID, rospy.Time(0))
            # convert to Pose
            current_pose = tfstamped2posestamped(current_pose)
            return current_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def get_dist(self, pose1, pose2):
        return np.linalg.norm(pose2np(pose1) - pose2np(pose2))

    def get_yaw_diff(self, quaternion_msg1, quaternion_msg2):
        return np.abs(quaternion_to_euler(quaternion_msg1)[2] - quaternion_to_euler(quaternion_msg2)[2])

    def check_collision(self):
        for obstacle_pose in OBSTACLES.poses:
            # convert obstacle to drone frame
            obstacle = PoseStamped()
            obstacle.pose = obstacle_pose
            obstacle.header = OBSTACLES.header
            obstacle_drone_posestamped = self.posestamped_to_frame(obstacle, DRONE_FRAME_ID)
            if obstacle_drone_posestamped.pose.position.x < 0:
                continue
            # check if obstacle is further than the next waypoint in drone frame
            # convert next waypoint to drone frame
            next_waypoint = PoseStamped()
            next_waypoint.pose = self.get_next_waypoint()
            next_waypoint.header = WAYPOINTS_POSES.header
            next_waypoint_drone_posestamped = self.posestamped_to_frame(next_waypoint, DRONE_FRAME_ID)
            if obstacle_drone_posestamped.pose.position.x > next_waypoint_drone_posestamped.pose.position.x:
                continue
            # if next_waypoint is within self.drone_width 
            min_width_point = - self.drone_width / 2
            max_width_point = self.drone_width /2
            if obstacle_drone_posestamped.pose.position.y >= min_width_point and obstacle_drone_posestamped.pose.position.y <= max_width_point:
                return True
        return False

    def get_next_waypoint(self):
        return WAYPOINTS_POSES.poses[self.next_waypoint_idx]

    def get_next_setpoint(self, current_posestamped):
        if self.get_dist(current_posestamped.pose, self.get_next_waypoint()) < self.look_ahead:
            return self.orient_sp(self.get_next_waypoint())
        # compute a setpoint that is self.look_ahead meters away from the current pose.
        # this is the setpoint that we will try to get to.
        vec = pose2np(self.get_next_waypoint()) - pose2np(current_posestamped.pose)
        vec = vec / np.linalg.norm(vec)
        vec = vec * self.look_ahead
        sp = np2pose(pose2np(current_posestamped.pose) + vec)
        return self.orient_sp(sp)

    def get_current_sp(self):
        # MAIN LOOP
        # get the current pose.
        current_pose = self.get_current_pose() # PoseStamped() or None
        if current_pose is None:
            rospy.logwarn("Planner issue: No current pose")
            return self.current_sp
            
        # if we're not at the next setpoint yet, just return the current setpoint.
        if self.get_dist(current_pose.pose, self.current_sp) >= self.sp_radius or self.get_yaw_diff(current_pose.pose.orientation, self.current_sp.orientation) >= self.yaw_tolerance:
            # log dist and angle diff
            rospy.loginfo("Planner: Not at setpoint yet")
            rospy.loginfo("Planner: Dist: {}".format(self.get_dist(current_pose.pose, self.current_sp)))
            rospy.loginfo("Planner: Angle: {}".format(self.get_yaw_diff(current_pose.pose.orientation, self.current_sp.orientation)))
            return self.current_sp

        # if arrived at the setpoint, increment timer
        # check if we have been at the current setpoint for self.hold_time
        if self.start_time is None:
            self.start_time = rospy.Time.now()
        if rospy.Time.now() - self.start_time < rospy.Duration(self.hold_time):
            return self.current_sp
        
        # if we have been at the current setpoint for self.hold_time, reset timer and get next setpoint
        self.start_time = None
        
        # check if arrived at the next global waypoint
        if self.get_dist(current_pose.pose, self.get_next_waypoint()) < self.sp_radius:
            self.next_waypoint_idx += 1
            self.next_waypoint_idx %= len(WAYPOINTS_POSES.poses)

        # check if current orientation is correct. if not, keep trying to get to the oriented setpoint.
        if self.get_yaw_diff(current_pose.pose.orientation, self.current_sp.orientation) > self.yaw_tolerance:
            return self.orient_sp(current_pose)

        # check if there's an obstacle. if so, avoid.
        if OBSTACLES is not None:
            # sort obstacles by distance to current pose
            obstacles = sorted(OBSTACLES.poses, key=lambda x: self.get_dist(current_pose.pose, x))
            for obstacle in obstacles:
                # convert Obstacle from Pose to PoseStamped
                obstacle = PoseStamped()
                obstacle.pose = obstacle
                obstacle.header = OBSTACLES.header
                if self.check_collision(current_pose, self.get_next_waypoint(), obstacle):
                    return self.avoid_obstacle(obstacle)
        # if no obstacle, move closer to next waypoint by 1 meter
        self.current_sp = self.get_next_setpoint()
        return self.current_sp

def waypoints_cb(msg):
    global WAYPOINTS_POSES
    WAYPOINTS_POSES = msg

def obstacles_cb(msg):
    global OBSTACLES
    OBSTACLES = msg

if __name__=="__main__":
    rospy.init_node("planner_node")
    # subscribers
    rospy.Subscriber(WAYPOINTS_TOPIC, PoseArray, waypoints_cb)
    rospy.Subscriber(TRACKER_OUTPUT, PoseArray, obstacles_cb)
    # publishers
    SP_PUB = rospy.Publisher(PLANNER_TOPIC, Pose, queue_size=1)
    
    planner = CoolPlanner()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if WAYPOINTS_POSES is None:
            rospy.logwarn("No waypoints")
            continue
        if OBSTACLES is None:
            rospy.logwarn("No obstacles")
            continue
        current_sp = planner.get_current_sp()
        SP_PUB.publish(current_sp)
        rate.sleep()



# From WF
# # check distance to current waypoint
# dist = np.linalg.norm(
#     posestamped2np(self.get_current_pose_world())
#     - pose2np(self.waypoints_world[self.global_waypoint_idx])
# )
# # if within radius, increment timer
# if dist < self.radius and self.start_time is None:
#     rospy.loginfo("starting timer on waypoint {}".format(self.global_waypoint_idx))
#     self.start_time = rospy.get_time()
# # if timer exceeds hold_time, increment waypoint
# if (
#     self.start_time is not None
#     and rospy.get_time() - self.start_time > self.hold_time
# ):
#     self.global_waypoint_idx += 1
#     rospy.loginfo(
#         "waypoint {} reached, indexing to next waypoint".format(
#             self.global_waypoint_idx
#         )
#     )
#     # if waypoint index exceeds number of waypoints, reset to 0
#     if self.global_waypoint_idx >= len(self.waypoints_world):
#         self.global_waypoint_idx = 0
#         rospy.loginfo("Waypoints complete. Resetting to first waypoint.")
#     # reset timer
#     self.start_time = None