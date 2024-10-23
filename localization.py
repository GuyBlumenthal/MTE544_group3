import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

# Used to determine which type of robot is running (Either a simulation or in-lab)
SIM_RUN = 'SIM'
LAB_RUN = 'LAB'
RUN_TYPE = LAB_RUN

rawSensor = 0
class localization(Node):

    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")

        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
        if RUN_TYPE == SIM_RUN: # Turtlebot3 Simulation
            qos=QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=5,
            )
        elif RUN_TYPE == LAB_RUN: # In Lab Robot
            qos=QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=5,
            )
        else:
            raise Exception(f"Undefined QoSProfile for run type {RUN_TYPE}")

        odom_qos=qos


        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None

        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
        # ...
        else:
            print("This type doesn't exist", sys.stderr)


    def odom_callback(self, pose_msg):

        # TODO Part 3: Read x,y, theta, and record the stamp
        self.pose=[ ... ]

        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])

    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py

