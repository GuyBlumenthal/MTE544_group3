# Imports
import rclpy

from rclpy.node import Node
from rclpy.time import Time

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time


# Type of robot motion to create
CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

# Used to determine which type of robot is running (Either a simulation or in-lab)
SIM_RUN = 'SIM'
LAB_RUN = 'LAB'
RUN_TYPE = LAB_RUN

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        self.type=motion_type
        
        self.radius_=0.0
        
        # On init, no sensors are set up
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # Publish speed at 10Hz
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
                
        # Create loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        if RUN_TYPE == SIM_RUN: # Turtlebot3
            qos=QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=5,
            )
        elif RUN_TYPE == LAB_RUN:
            qos=QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=5,
            )

        else:
            raise Exception(f"Undefined QoSProfile for run type {RUN_TYPE}")

        # IMU subscription
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, qos)
        self.imu_initialized = True
        
        # ENCODER subscription
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
        self.odom_initialized = True
        
        # LaserScan subscription 
        self.laser_subscription = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos)
        self.laser_initialized = True
        
        # Set up main loop
        self.create_timer(0.1, self.timer_callback)

    # IMU Logger Callback
    def imu_callback(self, imu_msg: Imu):
        self.imu_logger.log_values([
            imu_msg.linear_acceleration.x,                  # acc_x
            imu_msg.linear_acceleration.y,                  # acc_y
            imu_msg.angular_velocity.z,                     # angular_z
            Time.from_msg(imu_msg.header.stamp).nanoseconds # stamp
        ])

    # Odometry Logger Callback       
    def odom_callback(self, odom_msg: Odometry):
        self.odom_logger.log_values([
            odom_msg.twist.twist.linear.x,                      # x
            odom_msg.twist.twist.linear.y,                      # y
            odom_msg.twist.twist.angular.z,                     # th  
            Time.from_msg(odom_msg.header.stamp).nanoseconds    # stamp
        ])
                
    # Laser Logger Callback
    def laser_callback(self, laser_msg: LaserScan):
        ranges = [str(i) for i in laser_msg.ranges]
        self.laser_logger.log_values([
            laser_msg.angle_increment,                          # angle_increment
            f"[{' : '.join(ranges)}]",                          # list of ranges
            Time.from_msg(laser_msg.header.stamp).nanoseconds,  # stamp
        ])
                
    # Main robot loop
    def timer_callback(self):            
        if not self.successful_init:
            if self.odom_initialized and self.laser_initialized and self.imu_initialized:
                self.init_time = self.get_seconds()
                self.successful_init=True
            else:
                return
        
        # Create empty velocity message, load depending on selected motion type
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
        else:
            print("Type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        # Publish the defined velocity
        self.vel_publisher.publish(cmd_vel_msg)
        
    def make_circular_twist(self):
        # Circular motion -> Constant linear and rotation velocity
        msg=Twist()

        msg.linear.x = 1.0
        msg.angular.z = 1.0
        
        return msg

    def make_spiral_twist(self):
        # Spiral motion -> Constant rotational velocity, increasing linear velocity
        msg=Twist()
        coeff = 0.1 # Determined empirically

        # Max out at linear speed = 2 for safety
        msg.linear.x = min(float(self.get_seconds() - self.init_time) * coeff, 5)
        msg.angular.z = 1.5

        return msg
    
    def make_acc_line_twist(self):
        # Linear motion -> Constant speed in a direction
        msg=Twist()
        
        msg.linear.x = 1.0
        msg.angular.z = 0.0

        return msg
    
    def get_seconds(self):
        return self.get_clock().now().seconds_nanoseconds()[0]

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
