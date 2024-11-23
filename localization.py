import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

from math import atan2, asin, sqrt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

M_PI=3.1415926535


# Used to determine which type of robot is running (Either a simulation or in-lab)
SIM_RUN = 'SIM'
LAB_RUN = 'LAB'
RUN_TYPE = LAB_RUN

odom_qos = {
    SIM_RUN: QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=5,
    ),
    LAB_RUN: QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=5,
    )
}[RUN_TYPE]

rawSensors=0
kalmanFilter=1

class localization(Node):

    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.odom_logger =Logger("odomPose.csv", ["x, y, th, stamp"])
        self.pose=None

        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self, dt):

        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)

        x= [0, 0, 0, 0, 0, 0]

        Q= 0.9 * np.eye(6)

        R= 0.5 * np.eye(4)

        P= np.eye(6) # initial covariance

        self.kf=kalman_filter(P,Q,R, x, dt)

        # TODO Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)

        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)

    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        imu_ax = imu_msg.linear_acceleration.x
        imu_ay = imu_msg.linear_acceleration.y
        z= np.array([odom_msg.twist.twist.linear.x,
                     odom_msg.twist.twist.angular.z,
                     imu_ax,
                     imu_ay])

        # Implement the two steps for estimation
        self.kf.predict()
        self.kf.update(z)

        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        x, y, th, w, v, vdot = xhat
        self.pose=np.array([x,
                            y,
                            th,
                            odom_msg.header.stamp])

        kf_vx, kf_w, kf_ax, kf_ay = self.kf.measurement_model()

        # TODO Part 4: log your data
        # log "imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"
        self.loc_logger.log_values([imu_ax, imu_ay, kf_ax, kf_ay, kf_vx, kf_w, x, y, Time.from_msg(odom_msg.header.stamp).nanoseconds])
        self.odom_logger.log_values([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            euler_from_quaternion(odom_msg.pose.pose.orientation),
            Time.from_msg(odom_msg.header.stamp).nanoseconds
        ])

    def odom_callback(self, pose_msg):

        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":

    init()

    LOCALIZER=localization()

    spin(LOCALIZER)
