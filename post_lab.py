
import numpy as np
import matplotlib.pyplot as plt

lab_data = lambda cat: [
    f"lab_data/{cat}/angular.csv",
    f"lab_data/{cat}/linear.csv",
    f"lab_data/{cat}/robot_pose.csv"
]

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

def Main():

    angular, linear, pose = lab_data("PID")

    with open(pose, 'r') as pose_f:
        plot_pose(pose_f)

    with open(linear, 'r') as angular_f:
        lines = [line.split(',') for line in angular_f.readlines()]
        e, e_dot, e_int, stamp = [
            [float(line[0]) for line in lines[1:]],
            [float(line[1]) for line in lines[1:]],
            [float(line[2]) for line in lines[1:]],
            [float(line[3])/1e9 for line in lines[1:]],
        ]

        plt.plot(stamp, e, label="P-ERR")
        plt.plot(stamp, e_int, label="I-ERROR")
        plt.plot(stamp, e_dot, label="D-ERR")

        plt.legend()
        plt.grid()
        plt.show()





if __name__ == "__main__":
    Main()