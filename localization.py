import sys

from utilities import Logger, euler_from_quaternion, run_qos
from rclpy.time import Time
from rclpy.node import Node

from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):

    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")

        odom_qos=run_qos

        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None

        if localizationType == rawSensor:
            self.odom_subscription = self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)


    def odom_callback(self, pose_msg):
        theta = euler_from_quaternion([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
        ])[2] # [ Pitch, Roll, Yaw] <- We want the Yaw (Around the Z axis)

        self.pose= [
            pose_msg.pose.pose.position.x,                      # x
            pose_msg.pose.pose.position.y,                      # y
            theta,                                              # th
        ]

        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])

    def getPose(self):
        return self.pose

if __name__ == "__main__":
    rclpy.init()

    localizer = localization()

    try:
        rclpy.spin(localizer)
    except KeyboardInterrupt:
        print("Exiting")
