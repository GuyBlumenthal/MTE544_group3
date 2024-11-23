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

NO_FLIP = NO_FLIP_X, NO_FLIP_Y = [1, 1]
FLIP_X = FLIP_Y = -1

runs = [
    [1, 5, (FLIP_X, FLIP_Y)],
    [5, 1, (FLIP_X, FLIP_Y)],
    [5, 5, (NO_FLIP_X, NO_FLIP_Y)],
    [5, 9, (FLIP_X, FLIP_Y)],
    [9, 5, (FLIP_X, FLIP_Y)],
]

def load_odom(Q, R, MODE, last_t=math.inf, FLIP=NO_FLIP):
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
                center = (data[ODOM_X][-1], data[ODOM_Y][-1], data[STAMP][-1])

            # Center the odom pose
            data[ODOM_X][-1] = FLIP[0] * (data[ODOM_X][-1] - center[0])
            data[ODOM_Y][-1] = FLIP[1] * (data[ODOM_Y][-1] - center[1])
            # data[STAMP][-1]  = data[STAMP][-1] - center[2]
            data[STAMP][-1] = data[STAMP][-1] / 1e9

            if data[STAMP][-1] >= last_t:
                break

    return data


def load_pose(Q, R, MODE, last_t=math.inf, FLIP=NO_FLIP):
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
        data['headers'] = lines[0].split(',')
        for line in lines[1:]:
            line = line.strip().split(',')[:-1]

            for item in HEADERS:
                data[item].append(float(line[item]))

    data[STAMP] = (np.array(data[STAMP]) - data[STAMP][0]) / 1e9
    return data

def get_col(data, col):
    return [
        row[col] for row in data
    ]

def Main(show=False):
    # Get the lowest timestamp
    last_t = math.inf

    # Ensure last time is synchronized
    if False:
        for Q, R, _ in runs:
            kf_spiral = load_pose(Q, R, SPIRAL)
            odom_spiral = load_odom(Q, R, SPIRAL)

            last_t = min([last_t, kf_spiral[STAMP][-1], odom_spiral[STAMP][-1]])

        print("")

    errors = []

    plt.figure()
    plt.suptitle("Robot Position during Spiral Trajectory")

    plotStart = 3
    plotEnd =5
    for i, run in enumerate(runs):
        print(i)
        Q, R, FLIP = run

        kf_spiral   = load_pose(Q, R, SPIRAL, last_t, FLIP)
        odom_spiral = load_odom(Q, R, SPIRAL, last_t, FLIP)

        # Plot position
        if plotStart <= i and i < plotEnd and show:
            # plt.suptitle(f"")

            ax = plt.subplot(2, plotEnd-plotStart, 1+i-plotStart)

            ax.plot(kf_spiral[KF_X], kf_spiral[KF_Y], label="Kalman")
            ax.plot(odom_spiral[ODOM_X], odom_spiral[ODOM_Y], label="Odom")

            ax.set_title(f"Position - Q 0.{Q}, R 0.{R}")
            ax.set_ylabel("Y [m]")
            ax.set_xlabel("X [m]")
            ax.legend()
            ax.grid()

            ax = plt.subplot(2, plotEnd-plotStart, plotEnd-plotStart + 1 + i-plotStart)

            for i in range(KF_Y):
                ax.plot(kf_spiral[STAMP], kf_spiral[i], label=kf_spiral['headers'][i].strip())

            ax.set_title(f"Errors - Q 0.{Q}, R 0.{R}")
            ax.set_ylabel("Error")
            ax.set_xlabel("T [s]")
            ax.legend()
            ax.grid()

        # Determine error
        eSum = 0
        counts = min(len(kf_spiral[STAMP]), len(odom_spiral[STAMP]))
        # align starting times between kf and odom logs
        odom_offset = 0
        kf_offset = 0
        # print(odom_spiral[STAMP])
        # print(kf_spiral[STAMP])
        if(odom_spiral[STAMP][0] < kf_spiral[STAMP][0]):
            for index in range(counts):
                if odom_spiral[STAMP][index] >= kf_spiral[STAMP][0]:
                    odom_offset = index
                    print(f"odom ts: {odom_spiral[STAMP][index]} kf ts: {kf_spiral[STAMP][0]} odom offset: {odom_offset}")
                    break
        else:
            for index in range(counts):
                if kf_spiral[STAMP][index] >= odom_spiral[STAMP][0]:
                    kf_offset = index
                    print(f"kf ts: {kf_spiral[STAMP][index]} odom ts: {odom_spiral[STAMP][0]} kf offset: {kf_offset}")
                    break
        counts = min(len(kf_spiral[STAMP]) - kf_offset, len(odom_spiral[STAMP]) - odom_offset)

        for index in range(counts):
            eSum = eSum + math.sqrt(
                pow(kf_spiral[KF_X][index+kf_offset] - odom_spiral[ODOM_X][index+odom_offset], 2) +
                pow(kf_spiral[KF_Y][index+kf_offset] - odom_spiral[ODOM_Y][index+odom_offset], 2)
            )

        errors.append(eSum / counts)

    if show:
        # plt.tight_layout()
        plt.subplots_adjust(hspace=0.4,wspace=0.2)
        plt.show()

    best_e, best_run = math.inf, None
    for error, run in zip(errors, runs):
        print(f"Run Q 0.{run[0]}, R 0.{run[1]} Error - {error:.3f}")

        if error < best_e:
            best_e = error
            best_run = run

    best_run = runs[2]
    print(f"Best run - Q 0.{best_run[0]}, R 0.{best_run[1]}")

    # Plot the point for best trajectory
    if show:
        Q, R, FLIP = best_run
        kf_point   = load_pose(Q, R, POINT, last_t, FLIP)
        odom_point   = load_odom(Q, R, POINT, last_t, [NO_FLIP_X, FLIP_Y])

        plt.figure()

        ax = plt.subplot(1, 1, 1)

        ax.plot(kf_point[KF_X], kf_point[KF_Y], label="Kalman Pose")
        # ax.plot(odom_point[ODOM_X], odom_point[ODOM_Y], label="Odom Pose")

        ax.set_title(f"Robot Position during Spiral Trajectory - Q 0.{Q}, R 0.{R}")
        ax.set_ylabel("Y [m]")
        ax.set_xlabel("X [m]")
        ax.legend()
        ax.grid()

        plt.show()


if __name__ == "__main__":
    Main(show=True)
