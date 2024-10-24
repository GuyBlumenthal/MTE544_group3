from math import atan2, asin, sqrt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

M_PI=3.1415926535


# Used to determine which type of robot is running (Either a simulation or in-lab)
SIM_RUN = 'SIM'
LAB_RUN = 'LAB'
RUN_TYPE = LAB_RUN

run_qos = {
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


class Logger:

    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):

        self.filename = filename

        with open(self.filename, 'w') as file:

            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "

            header_str+="\n"

            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:

            vals_str=""

            for value in values_list:
                vals_str+=f"{value}, "

            vals_str+="\n"

            file.write(vals_str)


    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):

        self.filename = filename


    def read_file(self):

        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break

            next(file)

            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')

                row=[]

                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)

        return headers, table



# Function Definition from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python
def euler_yaw_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return yaw_z # in radians


def calculate_linear_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    x1, y1, _, _ = current_pose
    x2, y2 = goal_pose
    error_linear= sqrt(
        pow(y2 - y1, 2) +
        pow(x2 - x1, 2)
    )

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    x1, y1, theta1, _ = current_pose
    x2, y2 = goal_pose

    theta2 = atan2(
        y2 - y1,
        x2 - x1
    )

    error_angular = theta2 - theta1

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    while error_angular > M_PI:
        error_angular -= 2 * M_PI
    while error_angular < -M_PI:
        error_angular += 2 * M_PI

    return error_angular
