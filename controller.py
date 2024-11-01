import numpy as np


from pid import PID_ctrl
from utilities import euler_yaw_from_quaternion, calculate_angular_error, calculate_linear_error, SIM_RUN, LAB_RUN, RUN_TYPE

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

# Saturation Parameters
max_linear_vel, max_angular_vel = { # m/s, rad/s
    SIM_RUN: [0.22, 2.48],
    LAB_RUN: [0.31, 1.90]
}[RUN_TYPE]

class controller:
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        # Change the controller here
        SELECTED_CONTROLLER = PI

        self.PID_linear=PID_ctrl(SELECTED_CONTROLLER, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(SELECTED_CONTROLLER, kap, kav, kai, filename_="angular.csv")


    def vel_request(self, pose, goal, status):
        # Calculate linear and angular errors from current to target position
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        # Get timestamp out of pose [X Y THETA STAMP]
        timestamp = pose[3]

        # Run control, giving error, timestamp and the robot status
        linear_vel=self.PID_linear.update([e_lin, timestamp], status)
        angular_vel=self.PID_angular.update([e_ang, timestamp], status)

        # Apply linear saturation limits
        if linear_vel > max_linear_vel:
            linear_vel = max_linear_vel
        elif linear_vel < -max_linear_vel:
            linear_vel = -max_linear_vel

        # Apply angular saturation limits
        if angular_vel > max_angular_vel:
            angular_vel = max_angular_vel
        elif angular_vel < -max_angular_vel:
            angular_vel = -max_angular_vel

        return linear_vel, angular_vel


class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):

        super().__init__(klp, klv, kli, kap, kav, kai)

    def vel_request(self, pose, listGoals, status):

        # Current current (Close) and final (Last) goal points
        goal=self.lookFarFor(pose, listGoals)
        finalGoal=listGoals[-1]

        # Calculate linear and angular errors
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        # Get timestamp out of pose [X Y THETA STAMP]
        timestamp = pose[3]

        # Run control, giving error, timestamp and the robot status
        linear_vel=self.PID_linear.update([e_lin, timestamp], status)
        angular_vel=self.PID_angular.update([e_ang, timestamp], status)

        # Apply linear saturation
        if linear_vel > max_linear_vel:
            linear_vel = max_linear_vel
        elif linear_vel < -max_linear_vel:
            linear_vel = -max_linear_vel

        # Apply angular saturation
        if angular_vel > max_angular_vel:
            angular_vel = max_angular_vel
        elif angular_vel < -max_angular_vel:
            angular_vel = -max_angular_vel

        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        poseArray=np.array([pose[0], pose[1]])
        listGoalsArray=np.array(listGoals)

        # Get the closest point in the trajectory using euclidian distance
        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        # Return the the third point after the closest point up to the end
        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
