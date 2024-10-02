# Imports
import rclpy

from rclpy.node import Node
from rclpy.time import Time

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

SIM_RUN = 'SIM'
LAB_RUN = 'LAB'
RUN_TYPE = SIM_RUN

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        if RUN_TYPE == SIM_RUN: # Turtlebot3
            qos=QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=5,
            )
        else:
            raise Exception(f"Undefined QoSProfile for run type {RUN_TYPE}")

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, qos)
        
        # ENCODER subscription

        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
        
        # LaserScan subscription 
        
        self.laser_subscription = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos)

        self.successful_init=True
        self.imu_initialized=True
        self.odom_initialized=True
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        list = ["imu"]
        list.append(Time.from_msg(imu_msg.header.stamp).nanoseconds)
        list.append(imu_msg.linear_acceleration.x)
        list.append(imu_msg.linear_acceleration.y)
        list.append(imu_msg.linear_acceleration.z)
        list.append(imu_msg.angular_velocity.x)
        list.append(imu_msg.angular_velocity.y)
        list.append(imu_msg.angular_velocity.z)
        
    def odom_callback(self, odom_msg: Odometry):
        list = ["odom"]
        list.append(Time.from_msg(odom_msg.header.stamp).nanoseconds)
        list.append(odom_msg.twist.twist.linear.x)
        list.append(odom_msg.twist.twist.linear.y)
        list.append(odom_msg.twist.twist.linear.z)
        list.append(odom_msg.twist.twist.angular.x)
        list.append(odom_msg.twist.twist.angular.y)
        list.append(odom_msg.twist.twist.angular.z)
                
    def laser_callback(self, laser_msg: LaserScan):
        list = ["laser"]
        list.append(Time.from_msg(laser_msg.header.stamp).nanoseconds)
        list.append(laser_msg.angle_increment)
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        msg=Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        scaler = 50
        msg.linear.x = 6 * (self.get_clock().now().seconds_nanoseconds()[0]**2 % scaler) / scaler
        msg.angular.z = 2.0
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        msg.linear.x = 1.0
        msg.linear.y = 1.0
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
