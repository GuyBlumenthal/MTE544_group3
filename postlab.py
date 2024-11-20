import numpy as np
import matplotlib.pyplot as plt
import math

IMU_AX  =    0
IMU_AY  =    1
KF_AX   =    2
KF_AY   =    3
KF_VX   =    4
KF_W    =    5
KF_X    =    6
KF_Y    =    7

ODOM_X  =    0
ODOM_Y  =    1
ODOM_TH  =    2

STAMP   =   -1

POINT = "POINT"
SPIRAL = "SPIRAL"

runs = [
    [1, 5],
    [5, 1],
    [5, 5],
    [5, 9],
    [9, 5],
]

def load_odom(Q, R, MODE):
    HEADERS = [
        ODOM_X,
        ODOM_Y,
        ODOM_TH,
        STAMP,
    ]

    data = { item: list() for item in HEADERS }

    center = None

    with open(f"Q0{Q}R0{R}_{MODE}/odomPose.csv") as f:
        lines = f.readlines()
        for line in lines[1:]:
            line = line.strip().split(',')[:-1]

            for item in HEADERS:
                data[item].append(float(line[item]))

            if center is None:
                center = (data[ODOM_X][-1], data[ODOM_Y][-1])

            # Center the odom pose
            data[ODOM_X][-1] = -1 * (data[ODOM_X][-1] - center[0])
            data[ODOM_Y][-1] = -1 * (data[ODOM_Y][-1] - center[1])

            data[STAMP][-1] = data[STAMP][-1] / 1e9

    return data


def load_pose(Q, R, MODE):
    HEADERS = [
        IMU_AX,
        IMU_AY,
        KF_AX,
        KF_AY,
        KF_VX,
        KF_W,
        KF_X,
        KF_Y,
        STAMP,
    ]

    data = { item: list() for item in HEADERS }

    with open(f"Q0{Q}R0{R}_{MODE}/robotPose.csv") as f:
        lines = f.readlines()
        for line in lines[1:]:
            line = line.strip().split(',')[:-1]

            for item in HEADERS:
                data[item].append(float(line[item]))

            data[STAMP][-1] = data[STAMP][-1] / 1e9

    return data

def get_col(data, col):
    return [
        row[col] for row in data
    ]

def Main():
    for Q, R in runs:

        kf_spiral = load_pose(Q, R, SPIRAL)
        odom_spiral   = load_odom(Q, R, SPIRAL)

        # point_data = load_pose(Q, R, POINT)

        # Plot position
        plt.figure()

        plt.plot(kf_spiral[KF_X], kf_spiral[KF_Y], label="Kalman Pose")
        plt.plot(odom_spiral[ODOM_X], odom_spiral[ODOM_Y], label="Odom Pose")

        plt.title("Robot Position during Spiral Trajectory")
        plt.ylabel("Y [m]")
        plt.xlabel("X [m]")
        plt.legend()

        plt.show()

        # Determine error
        counts = min(len(kf_spiral[STAMP]), len(odom_spiral[STAMP]))
        eSum = 0
        for index in range(counts):
            eSum = math.sqrt(
                pow(kf_spiral[KF_X][index] - odom_spiral[ODOM_X][index], 2) +
                pow(kf_spiral[KF_Y][index] - odom_spiral[ODOM_Y][index], 2)
            )

        e = eSum / counts
        print(f"Q 0.{Q}, R 0.{R} - {e}")

        return


if __name__ == "__main__":
    Main()
