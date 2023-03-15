# TODO account for offset from vicon markers to pixhawk center

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

STATE: str = "Init"  # Launch, Test, Land, Abort, Init
SETPOINT: Pose = Pose()
SETPOINT.position.x = 0
SETPOINT.position.y = 0
SETPOINT.position.z = 0
# set orientation to 0
SETPOINT.orientation.w = 1
SETPOINT.orientation.x = 0
SETPOINT.orientation.y = 0
SETPOINT.orientation.z = 0

WAYPOINTS: PoseArray = None
WAYPOINT_IDX: int = 0
WAYPOINTS_RECEIVED: bool = False

DRONE_POSE: PoseStamped = PoseStamped()
DRONE_POSE.pose.position.x = 0
DRONE_POSE.pose.position.y = 0
DRONE_POSE.pose.position.z = 0
# set orientation to 0
DRONE_POSE.pose.orientation.w = 1
DRONE_POSE.pose.orientation.x = 0
DRONE_POSE.pose.orientation.y = 0
DRONE_POSE.pose.orientation.z = 0

# bounding volume radius
RADIUS = 0.5  # meters

# how long to hold position after reaching a waypoint
HOLD_TIME = 2  # seconds

DEBUG: bool = True


class WaypointFollower:
    def __init__(self, radius: float = 0.5, hold_time: int = 2) -> None:
        self.waypoint_idx: int = 0
        self.radius = radius
        self.hold_time = hold_time
        self.hold_timer = 0
        self.last_time = rospy.get_time()

    def update_setpoint(self, drone_pose: PoseStamped):
        global SETPOINT
        if not WAYPOINTS_RECEIVED:
            print("Waiting for waypoints")
            return None

        # check distance to current waypoint
        dist = np.linalg.norm(
            np.array(
                [
                    drone_pose.pose.position.x,
                    drone_pose.pose.position.y,
                    drone_pose.pose.position.z,
                ]
            )
            - WAYPOINTS[self.waypoint_idx]
        )
        if DEBUG:
            print(f"Distance to waypoint: {dist}")
        if dist < self.radius:
            # account for hold time
            if self.hold_timer < self.hold_time:
                self.hold_timer += rospy.get_time() - self.last_time
            else:
                self.hold_timer = 0
                self.waypoint_idx += 1
                self.waypoint_idx %= len(WAYPOINTS)
                print("Reached waypoint. Moving to next waypoint.")

        # set setpoint to current waypoint
        SETPOINT.position.x = WAYPOINTS[WAYPOINT_IDX, 0]
        SETPOINT.position.y = WAYPOINTS[WAYPOINT_IDX, 1]
        SETPOINT.position.z = WAYPOINTS[WAYPOINT_IDX, 2]


# Callback handlers
def handle_launch():
    global STATE, SETPOINT
    STATE = "Launch"
    print("Launch Requested.")
    # set launch setpoint drone
    SETPOINT.position.x = 0
    SETPOINT.position.y = 0
    SETPOINT.position.z = 1
    SETPOINT.orientation.w = 1
    SETPOINT.orientation.x = 0
    SETPOINT.orientation.y = 0
    SETPOINT.orientation.z = 0


def handle_test():
    global STATE
    STATE = "Test"
    print("Test Requested.")


def handle_land():
    global STATE, SETPOINT
    STATE = "Land"
    print("Land Requested.")
    # set launch setpoint drone
    SETPOINT.position.x = 0
    SETPOINT.position.y = 0
    SETPOINT.position.z = 0


def handle_abort():
    global STATE, SETPOINT
    STATE = "Abort"
    print("Abort Requested.")
    # set setpoint to ground plane directly below drone
    SETPOINT.position.x = DRONE_POSE.pose.position.x
    SETPOINT.position.y = DRONE_POSE.pose.position.y
    SETPOINT.position.z = 0


# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()


def callback_test(request):
    handle_test()
    return EmptyResponse()


def callback_land(request):
    handle_land()
    return EmptyResponse()


def callback_abort(request):
    handle_abort()
    return EmptyResponse()


def callback_pose(msg: PoseStamped):
    global DRONE_POSE
    DRONE_POSE = msg


def callback_waypoints(msg):
    global WAYPOINTS_RECEIVED, WAYPOINTS
    if WAYPOINTS_RECEIVED:
        return
    print("Waypoints Received")
    WAYPOINTS_RECEIVED = True
    WAYPOINTS = np.empty((0, 3))
    for pose in msg.poses:
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        WAYPOINTS = np.vstack((WAYPOINTS, pos))


# Main node
def comm_node():
    global STATE, WAYPOINTS, WAYPOINTS_RECEIVED

    # Do not change the node name and service topics!
    name = "rob498_drone_02"  # Change 00 to your team ID
    rospy.init_node(name)
    srv_launch = rospy.Service(name + "/comm/launch", Empty, callback_launch)
    srv_test = rospy.Service(name + "/comm/test", Empty, callback_test)
    srv_land = rospy.Service(name + "/comm/land", Empty, callback_land)
    srv_abort = rospy.Service(name + "/comm/abort", Empty, callback_abort)
    sub_waypoints = rospy.Subscriber(
        name + "/comm/waypoints", PoseArray, callback_waypoints
    )
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback_pose)
    sp_pub = rospy.Publisher(
        "/mavros/setpoint_position/local", PoseStamped, queue_size=1
    )
    print("Services, subscribers, publishers initialized")

    wf = WaypointFollower(RADIUS, HOLD_TIME)

    while not rospy.is_shutdown():
        if STATE == "test":
            wf.update_setpoint(DRONE_POSE)
        # publish setpoint
        sp_pub.publish(SETPOINT)
        rospy.sleep(0.2)


if __name__ == "__main__":
    comm_node()
