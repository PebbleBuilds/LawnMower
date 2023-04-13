mport numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse

from constants import *
from waypoint_follower import WaypointFollower
from graph import DirectedGraph
from pose_utils import posestamped2np

class WaypointPublisher(DirectedGraph):
    def __init__(
        self,
        radius=0.05,
        waypoints=None,
    ):
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


    def set_waypoints(self, waypoints):
        if self.waypoints_received:
            return
        print("Setting waypoints: \n", waypoints)
        # convert from vicon_world frame to vicon_inertial frame
        self.waypoints_world = waypoints
        self.waypoints_received = True

    def set_state(self, state):
        assert state in [LAUNCH, TEST, LAND, ABORT], "Invalid state " + state
        self.state = state

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
    graph.add_waypoints([posestamped2np(pose for pose in msg.poses)])


def planning_node():
    global graph
    # Do not change the node name and service topics!
    rospy.init_node("planner")
    # TODO change params to get from launch file
    graph = DirectedGraph()
   
    # subscribers
    rospy.Subscriber(WAYPOINTS_TOPIC, PoseArray, callback_waypoints)


    # publishers
    sp_pub = rospy.Publisher(PLANNER, PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        """
        Planning loic v1
        while me not at the end:
            me turn to next waypoint
            if me see obstacle:`
                me detect obstacle position
                me detect obstacle type
                me add obstacle(center, is_clockwise)
                me do dijkstra to next waypoint
                me get path
                me convert path coordinates to waypoints
                me do WF.set_waypoints(path_wpts)
            me go BRRRRRR!
        """
        
        setpoint = WF.get_setpoint()
        if setpoint is not None:
            sp_pub.publish(setpoint)
        rospy.sleep(0.2)


if __name__ == "__main__":
    planning_node()
