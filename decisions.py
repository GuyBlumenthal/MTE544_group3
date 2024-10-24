# Imports


import sys

from utilities import calculate_angular_error, calculate_linear_error, run_qos
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


MARGIN = 0.1


class decision_maker(Node):

    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):
        super().__init__("decision_maker")

        # Publisher for robot motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, rate)

        publishing_period=1/rate

        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)


        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):

        # # TODO Part 3: Run the localization node
        # ...    # Remember that this file is already running the decision_maker node.

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()

        # Check if you reached the goal
        if type(self.goal) == list:
            goal_point = self.goal[-1]
        else:
            goal_point = self.goal
        reached_goal = calculate_linear_error(self.localizer.getPose(), goal_point) < MARGIN


        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)

            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()

            raise SystemExit

        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # Publish the command to the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate

        self.publisher.publish(vel_msg)

import argparse


def main(args=None):

    init()

    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    odom_qos=run_qos

    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [2, 2])
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [0, 0], motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)

    while True:
        try:
            spin_once(DM)
            spin_once(DM.localizer)
        except SystemExit:
            print(f"reached there successfully {DM.localizer.pose}")
            break
        except KeyboardInterrupt:
            break


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
