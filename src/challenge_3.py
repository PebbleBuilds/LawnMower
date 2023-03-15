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

DRONE_POSE: PoseStamped = None

# bounding volume radius
RADIUS = 0.5  # meters

# how long to hold position after reaching a waypoint
HOLD_TIME = 2  # seconds

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
    global DRONE_POSE, STATE, SETPOINT, WAYPOINTS_RECEIVED, WAYPOINTS, WAYPOINT_IDX
    STATE = "Test"
    print("Test Requested.")
    if not WAYPOINTS_RECEIVED:
        print("No waypoints received. Holding position.")
        return
    # check distance to current waypoint
    dist = np.linalg.norm(
        np.array(
            [
                DRONE_POSE.pose.position.x,
                DRONE_POSE.pose.position.y,
                DRONE_POSE.pose.position.z,
            ]
        )
        - WAYPOINTS[WAYPOINT_IDX]
    )
    # TODO check if drone has been close to waypoint for a while
    if dist < RADIUS:
        # if close enough, move to next waypoint
        WAYPOINT_IDX += 1
        WAYPOINT_IDX %= len(WAYPOINTS)  # wrap around
        print("Reached waypoint. Moving to next waypoint.")
    # set setpoint to current waypoint
    SETPOINT.position.x = WAYPOINTS[WAYPOINT_IDX, 0]
    SETPOINT.position.y = WAYPOINTS[WAYPOINT_IDX, 1]
    SETPOINT.position.z = WAYPOINTS[WAYPOINT_IDX, 2]


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

    while not rospy.is_shutdown():
        if WAYPOINTS_RECEIVED:
            print("Waypoints:\n", WAYPOINTS)

        # Your code goes here
        if STATE == "Launch":
            print("Comm node: Launching...")
        elif STATE == "Test":
            print("Comm node: Testing...")
        elif STATE == "Land":
            print("Comm node: Landing...")
        elif STATE == "Abort":
            print("Comm node: Aborting...")
        # publish setpoint
        rospy.sleep(0.2)


if __name__ == "__main__":
    comm_node()
