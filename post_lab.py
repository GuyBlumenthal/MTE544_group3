import numpy as np
import matplotlib.pyplot as plt
import math

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


    # Determine start time
    index_0 = round(0.95 * [abs(item) > 0.01 for item in linear_data[EDOT]].index(True))

    for item in angular_data:
        angular_data[item] = angular_data[item][index_0:]

    for item in linear_data:
        linear_data[item] = linear_data[item][index_0:]

    for item in pose_data:
        pose_data[item] = pose_data[item][index_0:]

    angular_data[T] = np.array(angular_data[T]) - angular_data[T][0]
    linear_data[T] = np.array(linear_data[T]) - linear_data[T][0]
    pose_data[T] = np.array(pose_data[T]) - pose_data[T][0]

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

def TrajectoryPlotter():
    trajectories = ["Parabola", "Sigmoid"]

    for index, trajectory in enumerate(trajectories):
        angular_data, linear_data, pose_data = GetLabData(trajectory)

        ax = plt.subplot(1, len(trajectories), index + 1)

        ax.plot(pose_data[X], pose_data[Y])

        ax.set_title(f"Robot Trajectory for {trajectory} Trajectory")
        ax.set_ylabel("Y")
        ax.set_xlabel("X")
        ax.grid()

    plt.suptitle("Trajectories Using PI Controller")
    plt.tight_layout()
    plt.show()

def FindOvershootValue(list, ss):
    # find zero crossing at the steady state (ss)
    for i in range(len(list)):
        if (ss-list[i]) < 0:
            start = int(i - 0.05*len(list))
            # Plots for debugging
            # plt.plot(range(start, len(list)), list[start:])
            # plt.grid()
            # plt.show()

            # get largest negative value after zero crossing (overshoot)
            return (1 - abs(np.min(list[i:]) / ss)) * 100
    return 0

def FindOvershootPose(pose_list):
    ss_x = pose_list[X][-1]
    ss_y = pose_list[Y][-1]
    ss_th = pose_list[TH][-1]
    os_x = FindOvershootValue(pose_list[X], ss_x)
    os_y = FindOvershootValue(pose_list[Y], ss_y)
    os_th = FindOvershootValue(pose_list[TH], ss_th)
    return os_x, os_y, os_th

def FindSettlingTime(data_list, ess, margin):
    Ts = 0
    # find zero crossing
    for i in range(len(data_list[E])):
        if abs((data_list[E][i] - ess)/ess) > margin:
            Ts = data_list[T][i]
    return Ts


def CalculateParameters():
    # Load P data
    # Load PID data
    P_angular_data, P_linear_data, P_pose_data = GetLabData("P")
    PID_angular_data, PID_linear_data, PID_pose_data = GetLabData("PID")

    # Compare Agility, Accuracy, Overshoot for the POINT controller
    # Accuracy -> Steady State Error
    ess_ang_P = P_angular_data[E][-1]
    ess_lin_P = P_linear_data[E][-1]
    ess_ang_PID = PID_angular_data[E][-1]
    ess_lin_PID = PID_linear_data[E][-1]
    print(f"Steady state error, P angular: {ess_ang_P} linear {ess_lin_P} PID angular: {ess_ang_PID} linear {ess_lin_PID}")
    # Agility -> Settle time
    Ts_ang_P = FindSettlingTime(P_angular_data, ess_ang_P, (0.015-ess_ang_P)/ess_ang_P)
    Ts_lin_P = FindSettlingTime(P_linear_data, ess_lin_P, 0.1)
    Ts_ang_PID = FindSettlingTime(PID_angular_data, ess_ang_PID, (0.023-ess_ang_PID)/ess_ang_PID)
    Ts_lin_PID = FindSettlingTime(PID_linear_data, ess_lin_PID, 0.1)
    print(f"Settling time, P angular: {Ts_ang_P} linear: {Ts_lin_P} PID angular: {Ts_ang_PID} linear: {Ts_lin_PID}")
    # Overshoot -> %OS
    os_x_P, os_y_P , os_th_P = FindOvershootPose(P_pose_data)
    os_x_PID, os_y_PID, os_th_PID = FindOvershootPose(PID_pose_data)
    print(f"Overshoot, P x: {os_x_P} y: {os_y_P} th: {os_th_P} PID x: {os_x_PID} y: {os_y_PID} th: {os_th_PID}")


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

    TrajectoryPlotter()

    CalculateParameters()

if __name__ == "__main__":
    Main()