#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import PositionTarget
from std_srvs.srv import Empty, EmptyResponse

# position constants
SET_ALT = 1.44   ## should be 1.5

# mode constants
TAKEOFF = 0
LAND = 1
TEST = 2
ABORT = 3

RATE = 20

class CommNode:
    # Main communication node for ground control
    def __init__(self, node_name="rob498_drone_02"):
        self.node_name = node_name
        rospy.init_node(node_name)

        # initialize services
        # Change service defs to node_name+'/comm/name' when they add team names
        self.srv_launch = rospy.Service(
           self.node_name + "/comm/launch", Empty, self.launch_cb
        )
        self.srv_test = rospy.Service(self.node_name + "/comm/test", Empty, self.test_cb)
        self.srv_land = rospy.Service( self.node_name + "/comm/land", Empty, self.land_cb)
        self.srv_abort = rospy.Service(self.node_name + "/comm/abort", Empty, self.abort_cb)
        self.mode = None
        self.rate = rospy.Rate(RATE)
        print("services intiialized")

        # initialize subscribers
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.posCb)

        # initialize publishers
        self.sp_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=1
        )

        # initialize setpoints
        self.set_point = PoseStamped()

        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 1.44
        # self.goal_z = 0

        # initialize set_point position
        self.set_point.pose.position.x = self.goal_x
        self.set_point.pose.position.y = self.goal_y
        self.set_point.pose.position.z = self.goal_z

        # initialize set_point orientation
        self.set_point.pose.orientation.w = 1
        self.set_point.pose.orientation.x = 0
        self.set_point.pose.orientation.y = 0
        self.set_point.pose.orientation.z = 0

        # intialize local_pos position
        self.local_pos = Point(0, 0, 0)


    def posCb(self, msg):
        # update local position
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    def updateSp(self):
        # update setpoint being published
        self.set_point.pose.position.x = self.goal_x
        self.set_point.pose.position.y = self.goal_y
        self.set_point.pose.position.z = self.goal_z

    # Callback handlers
    def handle_launch(self):
        print("Launch Requested. Your drone should take off.")
        self.goal_z = SET_ALT
        self.mode = TAKEOFF

    def handle_test(self):
        print(
            "Test Requested. Your drone should perform the required tasks. Recording starts now."
        )
        self.goal_z = SET_ALT
        self.mode = TEST

    def handle_land(self):
        print("Land Requested. Your drone should land.")
        self.goal_z = 0
        self.mode = LAND

    def handle_abort(self):
        print(
            "Abort Requested. Your drone should land immediately due to safety considerations"
        )
        self.goal_z = 0
        self.mode = ABORT

    # Service callbacks
    def launch_cb(self, request):
        self.handle_launch()
        return EmptyResponse()

    def test_cb(self, request):
        self.handle_test()
        return EmptyResponse()

    def land_cb(self, request):
        self.handle_land()
        return EmptyResponse()

    def abort_cb(self, request):
        self.handle_abort()
        return EmptyResponse()

    def spin(self):
        while not rospy.is_shutdown():
            # update setpoints
            self.updateSp()
            # publish setpoints
            self.sp_pub.publish(self.set_point)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        comm_node = CommNode()
        comm_node.spin()
    except rospy.ROSInterruptException:
        pass
