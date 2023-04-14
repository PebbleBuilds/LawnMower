import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse
import tf2_ros

from pose_utils import create_posestamped, posestamped2np, tfstamped2posestamped
from constants import *

from constants import *
from waypoint_follower import WaypointFollower
from graph import DirectedGraph
from pose_utils import posestamped2np, euler_to_quaternion, get_yaw

class WaypointPlanner(DirectedGraph):
    def __init__(
        self,
        radius=0.05,
        launch_height = 1,
        waypoints=None,
    ):
        DirectedGraph.__init__(self) 
        self.waypoints_received = False
        if waypoints is not None:
            self.set_waypoints(waypoints)
        self.radius = radius

        # tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # initialize timers and waypoint index
        self.global_waypoint_idx = 0
        self.dance_waypoint_idx = 0
        self.waypoint_arrival_time = None

        # next obstacle params
        self.next_obstacle_pos = None
        self.next_obstacle_type = None

        # waypoint queue and params
        self.launch_wpt = create_posestamped(
            [0, 0, launch_height],
            orientation=[0, 0, -0.7071068, 0.7071068],  # checkerboard wall
            frame_id=VICON_ORIGIN_FRAME_ID,
        )
        self.current_waypoint = None
        self.next_waypoint = None

    def get_current_pose_world(self):
        try:
            t = self.listener.lookupTransform(
                VICON_ORIGIN_FRAME_ID, DRONE_FRAME_ID, rospy.Time(0)
            )
            return tfstamped2posestamped(t)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.loginfo(
                "Waiting for transform from vicon to local origin. Returning last setpoint."
            )
            return None


def callback_waypoints(msg):
    WP.add_waypoints([posestamped2np(pose for pose in msg.poses)])

def callback_obstacle_pos(msg)
    WP.obstacles_poses = [pose2np(pose) for pose in msg]

def callback_obstacle_type(msg);
    WP.next_obstacle_type = msg


def planning_node():
    global WP
    # Do not change the node name and service topics!
    rospy.init_node("planner")
    # TODO change params to get from launch file
    WP = WaypointPlanner()
   
    # subscribers
    rospy.Subscriber(WAYPOINTS_TOPIC, PoseArray, callback_waypoints)
    rospy.Subscriber(OBSTACLE_POS_TOPIC, PoseStamped, callback_obstacle_pos)
    # rospy.Subscriber(OBSTACLE_TYPE_TOPIC, PoseStamped, callback_obstacle_type)


    # publishers
    wp_pub = rospy.Publisher(PLANNER, PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        if WP.current_waypoint is None: # If going to launch waypoint
            WP.next_waypoint = WP.launch_waypoint
        
        dist = np.linalg.norm(
            posestamped2np(WP.get_current_pose_world())
            - posestamped2np(WP.next_waypoint)
        ) 
        
        if dist < WP.radius:
            WP.update_obstacles(WP.obstacles_poses, INITIAL_OBSTACLE_CLOCKWISE)
            WP.modify_path(WP.current_waypoint, WP.get_next_wpt)
            WP.current_waypoint = WP.next_waypoint
            WP.next_waypoint = WP.get_next_wpt(WP.current_waypoint)
        
        if WP.next_waypoint in WP.main_waypoints:
            next_orientation = euler_to_quaternion(get_yaw(WP.current_waypoint[:2]
                                                    WP.next_waypoint[:2]))

        self.next_setpoint = create_posestamped(
            WP.next_waypoint,
            orientation=next_orientation,  # checkerboard wall
            frame_id=VICON_ORIGIN_FRAME_ID,
        )

        if WP.next_waypoint != WP.main_waypoints[-1]:
            wp_pub.publish(WP.next_waypoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    planning_node()
