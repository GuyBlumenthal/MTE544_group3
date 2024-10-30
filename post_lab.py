
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

def plot_pose(file):
    lines = [line.split(',') for line in file.readlines()]
    x, y, t = [
        [float(line[0]) for line in lines[1:]],
        [float(line[1]) for line in lines[1:]],
        [float(line[2]) for line in lines[1:]],
    ]

    traj_x, traj_y = trajectory_planner()
    plt.scatter(x, y)
    plt.plot(traj_x, traj_y, label="Ref")
    plt.legend()
    plt.show()

def ControllerPlot(controller_name):
    angularF, linearF, poseF = lab_data(controller_name)

    lin_e, lin_edot = [], []

    x, y, theta = [], [], []

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


    plt.plot(pose_data[X], pose_data[Y])
    plt.show()

def Main():

    # Required plots:
    # For each P, PID
    #   e, e_dot vs T of linear
    #   e, e_dot vs T of angular
    #   x, y, theta vs T
    #   x vs y
    #

    ControllerPlot("P")
    ControllerPlot("PID")

if __name__ == "__main__":
    Main()