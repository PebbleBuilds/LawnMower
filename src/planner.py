mport numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse

from constants import *
from waypoint_follower import WaypointFollower
from graph import DirectedGraph
from pose_utils import posestamped2np

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

    def handle_test(self):
        # TODO reimplement dancing
        # check distance to current waypoint
        dist = np.linalg.norm(
            posestamped2np(self.get_current_pose_world())
            - posestamped2np(self.waypoints_world[self.global_waypoint_idx])
        )
        # if within radius, increment timer
        if dist < self.radius and self.start_time is None:
            rospy.loginfo("starting timer on waypoint", self.global_waypoint_idx)
            self.start_time = rospy.get_time()
        # if timer exceeds hold_time, increment waypoint
        if (
            self.start_time is not None
            and rospy.get_time() - self.start_time > self.hold_time
        ):
            self.global_waypoint_idx += 1
            rospy.loginfo(
                "waypoint {} reached, indexing to next waypoint".format(
                    self.global_waypoint_idx
                )
            )
            # if waypoint index exceeds number of waypoints, reset to 0
            if self.global_waypoint_idx >= self.waypoints_world.shape[0]:
                self.global_waypoint_idx = 0
                rospy.loginfo("Waypoints complete. Resetting to first waypoint.")
            # reset timer
            self.start_time = None
        setpoint = self.waypoints_world[self.global_waypoint_idx]
        return setpoint


def callback_waypoints(msg):
    WP.add_waypoints([posestamped2np(pose for pose in msg.poses)])

def callback_obstacle_pos(msg)
    # WP.add_obstacle(posestamped2np(pose), )
    self.next_obstacle_pos = msg

def callback_obstacle_type(msg);
    self.next_obstacle_type = msg


def planning_node():
    global WP
    # Do not change the node name and service topics!
    rospy.init_node("planner")
    # TODO change params to get from launch file
    WP = WaypointPlanner()
   
    # subscribers
    rospy.Subscriber(WAYPOINTS_TOPIC, PoseArray, callback_waypoints)
    # rospy.Subscriber(OBSTACLE_POS_TOPIC, PoseStamped, callback_obstacle_pos)
    # rospy.Subscriber(OBSTACLE_TYPE_TOPIC, PoseStamped, callback_obstacle_type)


    # publishers
    sp_pub = rospy.Publisher(PLANNER, PoseStamped, queue_size=1)

    while not rospy.is_shutdown():        
        if WP.next_obstacle_pos not in WP.obstacles:
            rospy.loginfo("Adding obstacle at %s of type %r with radius %d and effective radius %d" % (str(WP.next_obstacle_pos), WP.next_obstacle_type, RADIUS, FOS*RADIUS)
)
            WP.add_obstacle(WP.next_obstacle_pos, OBS_RADIUS, WP.next_obstacle_type,FOS)
            WP.modify_path(WP.waypoints[0], WP.waypoints[-1])
        
        if WP.current_waypoint is None: # If going to launch waypoint
            WP.next_waypoint = WP.launch_waypoint
        
        dist = np.linalg.norm(
            posestamped2np(WP.get_current_pose_world())
            - posestamped2np(WP.next_waypoint)
        ) 
        
        if dist < WP.radius:
            WP.current_waypoint = WP.next_waypoint
            WP.next_waypoint = WP.get_next_wpt(WP.current_waypoint)
        
        sp_pub.publish(WP.next_waypoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    planning_node()
