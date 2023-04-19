#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion
import tf2_ros
import copy
from constants import *
import numpy as np
from pose_utils import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_geometry_msgs

WAYPOINTS_POSES = None # PoseArray
OBSTACLES = None # PoseArray

OBSTACLE_CLOCKWISE = [True, False, True, False] # red is clockwise green is anticlockwise

class CoolPlanner():
    def __init__(self):
        self.current_sp = Pose()
        self.current_sp.position.z = 1
        self.current_sp.orientation.w = 1
        self.next_waypoint_idx = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sp_radius = 0.2 # meters
        self.yaw_tolerance = 0.4 # radians
        self.hold_time = 2.0
        self.drone_radius = 0.5 # meters
        self.look_ahead = 1.0
        self.avoid_dist = 0.5 # metres
        self.start_time = None


    def orient_sp(self, curr_pose):
        # get the next waypoint
        next_waypoint = self.get_next_waypoint()
        # calculate the angle
        y = next_waypoint.position.y - curr_pose.position.y
        x = next_waypoint.position.x - curr_pose.position.x
	    # flip the sign because we saw it turning backwards
        theta = np.arctan2(y,x)
        # create a new posestamped
        orientation = quaternion_from_euler(0.0,0.0,theta)
        new_quat = Quaternion(
            x=orientation[0], 
            y=orientation[1], 
            z=orientation[2], 
            w=orientation[3])
        return new_quat

    def posestamped_to_frame(self, posestamped, frame_id):
        try:
            posestamped_drone = self.tf_buffer.transform(posestamped, frame_id, rospy.Duration(1))
            return posestamped_drone
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            print(e)
            rospy.loginfo(
                "Waiting for transform from vicon to local origin. Returning given posestamped."
            )
            return None

    def avoid_obstacle(self, obstacle_posestamped, current_posestamped):
        # check which quadrant the obstacle is in world frame
        if obstacle_posestamped.pose.position.x > 0:
            if obstacle_posestamped.pose.position.y > 0:
                quadrant = 0
            else:
                quadrant = 1
        else:
            if obstacle_posestamped.pose.position.y > 0:
                quadrant = 3
            else:
                quadrant = 2
        left = OBSTACLE_CLOCKWISE[quadrant]
        # convert obstacle to drone frame
        obstacle_pose_d = self.posestamped_to_frame(obstacle_posestamped, DRONE_FRAME_ID)
        if obstacle_pose_d is None:
            rospy.logwarn("obstacle pose is None")
            return None
        avoid_sp_d = copy.deepcopy(obstacle_pose_d)
        if left:
            avoid_sp_d.pose.position.y = avoid_sp_d.pose.position.y + self.avoid_dist
        else:
            avoid_sp_d.pose.position.y = avoid_sp_d.pose.position.y - self.avoid_dist
        sp = self.posestamped_to_frame(avoid_sp_d, VICON_DUMMY_FRAME_ID)
        if sp is None:
            rospy.logwarn("avoid sp is None")
            return None
        sp = sp.pose
        orientation =  self.orient_sp(sp)
        sp.position.z = current_posestamped.pose.position.z
        sp.orientation = orientation
        return sp

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
        # convert msg into list
        quat1 = [quaternion_msg1.x, quaternion_msg1.y, quaternion_msg1.z, quaternion_msg1.w]
        quat2 = [quaternion_msg2.x, quaternion_msg2.y, quaternion_msg2.z, quaternion_msg2.w]
        return np.abs(euler_from_quaternion(quat1)[2] - euler_from_quaternion(quat2)[2])

    def check_collision(self, obstacle_drone_posestamped, next_waypoint_drone_posestamped):
        # print("checking collision with obstacle at", obstacle_drone_posestamped)
        if obstacle_drone_posestamped.pose.position.x < 0.25:
            return False
        if obstacle_drone_posestamped.pose.position.x > next_waypoint_drone_posestamped.pose.position.x:
            # rospy.loginfo("obstacle is past next waypoint: {} > {}".format(obstacle_drone_posestamped.pose.position.x, next_waypoint_drone_posestamped.pose.position.x))
            return False
        # if next_waypoint is within self.drone_width 
        min_width_point = - self.drone_radius
        max_width_point = self.drone_radius
        return obstacle_drone_posestamped.pose.position.y >= min_width_point and obstacle_drone_posestamped.pose.position.y <= max_width_point

    def get_next_waypoint(self):
        return WAYPOINTS_POSES.poses[self.next_waypoint_idx]

    def get_next_setpoint(self, current_posestamped):
        if self.get_dist(current_posestamped.pose, self.get_next_waypoint()) < self.look_ahead:
            orientation = self.orient_sp(current_posestamped.pose)
            sp = self.get_next_waypoint()
            sp.orientation = orientation
            rospy.loginfo("continue directly to next waypoint")
            rospy.loginfo(sp)
            return sp
        rospy.loginfo("setpoint is look ahead")
        # compute a setpoint that is self.look_ahead meters away from the current pose.
        # this is the setpoint that we will try to get to.
        vec = pose2np(self.get_next_waypoint()) - pose2np(current_posestamped.pose)
        vec = vec / np.linalg.norm(vec)
        vec = vec * self.look_ahead
        sp = np2pose(pose2np(current_posestamped.pose) + vec)
        sp.orientation = self.orient_sp(sp)
        return sp

    def get_current_sp(self):
        # MAIN LOOP
        # get the current pose.
        current_posestamped = self.get_current_pose() # PoseStamped() or None
        if current_posestamped is None:
            rospy.logwarn("Planner issue: No current pose")
            return self.current_sp
        current_stamp = current_posestamped.header.stamp
        # if we're not at the next setpoint yet, just return the current setpoint.
        if self.get_dist(current_posestamped.pose, self.current_sp) >= self.sp_radius:
            # rospy.loginfo("Planner: Not at setpoint yet")
            # rospy.loginfo("Planner: Dist: {}".format(self.get_dist(current_posestamped.pose, self.current_sp)))
            return self.current_sp
        if self.get_yaw_diff(current_posestamped.pose.orientation, self.current_sp.orientation) >= self.yaw_tolerance:
            # log dist and angle diff
            # rospy.loginfo("Planner: Angle: {}".format(self.get_yaw_diff(current_posestamped.pose.orientation, self.current_sp.orientation)))
            return self.current_sp

        # if arrived at the setpoint, increment timer
        # check if we have been at the current setpoint for self.hold_time
        if self.start_time is None:
            self.start_time = rospy.Time.now()
        if rospy.Time.now() - self.start_time < rospy.Duration(self.hold_time):
            rospy.loginfo("Planner: holding at setpoint, diff of {}".format(rospy.Time.now().to_sec() - self.start_time.to_sec()))
            return self.current_sp

        # if we have been at the current setpoint for self.hold_time, reset timer and get next setpoint
        self.start_time = None
        rospy.loginfo("Planner: Arrived at setpoint, getting next setpoint")

        # check if arrived at the next global waypoint
        if self.get_dist(current_posestamped.pose, self.get_next_waypoint()) < self.sp_radius:
            rospy.loginfo("reached waypoint {}".format(self.next_waypoint_idx))
            self.next_waypoint_idx += 1
            self.next_waypoint_idx %= len(WAYPOINTS_POSES.poses)
            rospy.loginfo("turning to face next waypoint")
            self.current_sp.orientation = self.orient_sp(current_posestamped.pose)
            return self.current_sp

        # check if there's an obstacle. if so, avoid.
        if OBSTACLES is not None:
            # sort obstacles by distance to current pose
            obstacles_sorted = sorted(OBSTACLES.poses, key=lambda x: self.get_dist(current_posestamped.pose, x))
            # convert next waypoint to drone frame
            next_waypoint = PoseStamped()
            next_waypoint.pose = self.get_next_waypoint()
            next_waypoint.header = WAYPOINTS_POSES.header
            next_waypoint.header.stamp = current_stamp
            next_waypoint_drone_posestamped = self.posestamped_to_frame(next_waypoint, DRONE_FRAME_ID)
            if next_waypoint_drone_posestamped is None:
                rospy.logwarn("Planner issue: next_waypoint_drone_posestamped is None")
                return self.current_sp
            for obstacle in obstacles_sorted:
                obstacle_posestamped = PoseStamped()
                obstacle_posestamped.pose = obstacle
                obstacle_posestamped.header = OBSTACLES.header
                obstacle_posestamped.header.stamp = current_stamp
                # check if obstacle is in front of the drone and within the drone width and closer than the next waypoint
                obstacle_drone_posestamped = self.posestamped_to_frame(obstacle_posestamped, DRONE_FRAME_ID)
                if obstacle_drone_posestamped is None:
                    rospy.logwarn("Planner issue: obstacle_drone_posestamped is None")
                    return self.current_sp
                if self.check_collision(obstacle_drone_posestamped, next_waypoint_drone_posestamped):
                    rospy.loginfo("Planner: Obstacle detected at {}".format(obstacle.position))
                    current_sp = self.avoid_obstacle(obstacle_posestamped, current_posestamped)
                    if current_sp is not None:
                        self.current_sp = current_sp
                    else:
                        rospy.logwarn("Planner issue: current_sp is None")
                    rospy.loginfo("avoiding obstacle, moving to")
                    rospy.loginfo(self.current_sp.position)
                    return self.current_sp

        # if no obstacle, move closer to next waypoint by 1 meter
        rospy.loginfo("Planner: No obstacle detected")
        self.current_sp = self.get_next_setpoint(current_posestamped)
        rospy.loginfo(self.current_sp)
        return self.current_sp

def waypoints_cb(msg):
    global WAYPOINTS_POSES
    if WAYPOINTS_POSES is None:
        rospy.loginfo("Got waypoints")
        rospy.loginfo(msg)
        WAYPOINTS_POSES = msg
        PLANNER.current_sp.orientation = PLANNER.orient_sp(PLANNER.current_sp)

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
    
    PLANNER = CoolPlanner()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if WAYPOINTS_POSES is None:
            rospy.logwarn("No waypoints")
            continue
        if OBSTACLES is None:
            rospy.logwarn("No obstacles")
            continue
        current_sp = PLANNER.get_current_sp()
        SP_PUB.publish(current_sp)
        rate.sleep()
