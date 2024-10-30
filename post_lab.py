
import numpy as np
import matplotlib.pyplot as plt

lab_data = lambda cat: [
    f"lab_data/{cat}/angular.csv",
    f"lab_data/{cat}/linear.csv",
    f"lab_data/{cat}/robot_pose.csv"
]

E = "e"
EDOT = "e_dot"
T = "Time"
X = "X"
Y = "Y"
TH = "Theta"

def trajectory_planner():
    traj = {
        "PARABOLA": {
            "min": 0,
            "max": 1.5,
            "func": lambda x: x**2,
        },
        "SIGMOID": {
            "min": 0,
            "max": 2.5,
            "func": lambda x: 2 / (1 + np.exp(-2 * x)) - 1
        }
    }["PARABOLA"]


    # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
    x = np.linspace(traj["min"], traj["max"], 10)
    y = np.vectorize(traj['func'])(x)
    return x, y

def GetLabData(folder):
    angularF, linearF, poseF = lab_data(folder)

    with open(angularF, 'r') as angular_raw:
        lines = [line.split(',') for line in angular_raw.readlines()]
        t_0 = float(lines[1][3])/1e9
        angular_data = {
            E: [float(line[0]) for line in lines[1:]],
            EDOT: [float(line[1]) for line in lines[1:]],
            T: [float(line[3])/1e9 - t_0 for line in lines[1:]],
        }

    with open(linearF, 'r') as linear_raw:
        lines = [line.split(',') for line in linear_raw.readlines()]
        t_0 = float(lines[1][3])/1e9
        linear_data = {
            E: [float(line[0]) for line in lines[1:]],
            EDOT: [float(line[1]) for line in lines[1:]],
            T: [float(line[3])/1e9 - t_0 for line in lines[1:]],
        }

    with open(poseF, 'r') as pose_raw:
        lines = [line.split(',') for line in pose_raw.readlines()]
        t_0 = float(lines[1][3])/1e9
        pose_data = {
            X:  [float(line[0]) for line in lines[1:]],
            Y:  [float(line[1]) for line in lines[1:]],
            TH: [float(line[2]) for line in lines[1:]],
            T: [float(line[3])/1e9 - t_0 for line in lines[1:]],
        }

    return angular_data, linear_data, pose_data


def ControllerPlot(controller_name):
    angular_data, linear_data, pose_data = GetLabData(controller_name)

    # Subplots TODO
    # |-----------------------|
    # |                       |
    # |  E,EDOTvsT  E,EDOTvsT |
    # |                       |
    # |       X,Y,TH vs T     |
    # |          XvsY         |
    # |                       |
    # |   EvsEDOT   EvsEDOT   |
    # |                       |
    # |-----------------------|
    plt.suptitle(f"Controller Data Plot for {controller_name} Controller")

    ax = plt.subplot(3, 2, 1)
    ax.plot(angular_data[T], angular_data[E], label="E")
    ax.plot(angular_data[T], angular_data[EDOT], label="E Dot")

    ax.set_title("Angular Error Data")
    ax.set_ylabel("Errors")
    ax.set_xlabel("Time")
    ax.legend()
    ax.grid()

    ax = plt.subplot(3, 2, 2)
    ax.plot(linear_data[T], linear_data[E], label="E")
    ax.plot(linear_data[T], linear_data[EDOT], label="E Dot")

    ax.set_title("Linear Error Data")
    ax.set_ylabel("Errors")
    ax.set_xlabel("Time")
    ax.legend()
    ax.grid()

    ax = plt.subplot(3, 2, 3)
    ax.plot(pose_data[T], pose_data[X], label="X")
    ax.plot(pose_data[T], pose_data[Y], label="Y")
    ax.plot(pose_data[T], pose_data[TH], label="Theta")

    ax.set_title("Robot State over Time")
    ax.set_ylabel("State")
    ax.set_xlabel("Time")
    ax.legend()
    ax.grid()

    ax = plt.subplot(3, 2, 4)
    ax.plot(pose_data[X], pose_data[Y], label="Pose")

    ax.set_title("Robot Trajectory")
    ax.set_ylabel("Y")
    ax.set_xlabel("X")
    ax.legend()
    ax.grid()

    ax = plt.subplot(3, 2, 5)
    ax.plot(angular_data[EDOT], angular_data[E])

    ax.set_title("Angular Error Plot")
    ax.set_ylabel("E")
    ax.set_xlabel("E Dot")
    ax.grid()

    ax = plt.subplot(3, 2, 6)
    ax.plot(linear_data[EDOT], linear_data[E])

    ax.set_title("Linear Error Plot")
    ax.set_ylabel("E")
    ax.set_xlabel("E Dot")
    ax.grid()

    plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.25)
    plt.show()

def TrajectoryPlotter(trajectory):
    angular_data, linear_data, pose_data = GetLabData(trajectory)

def CalculateParameters():
    # Load P data
    # Load PID data
    P_angular_data, P_linear_data, P_pose_data = GetLabData("P")
    PID_angular_data, PID_linear_data, PID_pose_data = GetLabData("PID")

    # Compare Agility, Accuracy, Overshoot for the POINT controller

    # Agility -> Settle time?
    # Accuracy -> Steady State Error
    # Overshoot -> %OS

    pass

def Main():

    # Required plots:
    # For each P, PID
    #   e, e_dot vs T of linear
    #   e, e_dot vs T of angular
    #   x, y, theta vs T
    #   x vs y
    #   e vs e_dot of linear
    #   e vs e_dot of angular

    ControllerPlot("P")
    ControllerPlot("PID")

    CalculateParameters()

if __name__ == "__main__":
    Main()