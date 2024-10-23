import os
from filePlotter import plot_errors
from utilities import Logger
import numpy as np
import matplotlib.pyplot as plt
from math import floor
import math

class SENSOR:
    IMU = "imu"
    ODOM = "odom"
    LASER = "laser"

class MOTION:
    LINE = "line"
    CIRCLE = "circle"
    SPIRAL = "spiral"

FILE = lambda sensor, motion: f"lab1_data/{sensor}_content_{motion}.csv"

def lpf(t, signal):
    if len(signal) <= 1:
        return signal

    window = 0.5 # seconds
    dt = (t[1] - t[0])
    print(dt)

    window = int(window / dt)
    print(window)

    print(window)
    np_signal = list(signal)
    for i in range(window):
        np_signal.append(signal[-1])

    result = []
    for i in range(len(signal)):
        bottom = max(0, i - window)
        result.append(np.average(np_signal[bottom:i]))

    return result


def loc_avg(t, signal):
    if len(signal) <= 1:
        return signal

    window = 0.5 # seconds
    dt = (t[1] - t[0])

    window = int(window / dt)
    print(window)

    np_signal = list(signal)
    for i in range(window):
        np_signal.append(signal[-1])

    result = []
    for i in range(len(signal)):
        bottom = int(max(0, i - window / 2))
        top = int(min(len(np_signal) - 1, i + window / 2))
        result.append(np.median(np_signal[bottom:top]))

    return result


def plot_imu(file, motion, i):
    a_x, a_y, omega_z, t = [], [], [], []

    with open(file, 'r') as f:
        lines = f.readlines()
        for line in lines[1:]:
            x, y, z, ct = [float(item) for item in line.split(',')]
            a_x.append(x)
            a_y.append(y)
            omega_z.append(z)
            t.append(ct / 1e9)

    t = np.array(t) - t[0]

    plot_def = lambda i, v, t, l, u: {
        'index': i,
        'value': v,
        'title': t,
        'label': l,
        'unit': u
    }

    plots = [
        plot_def(
            1,
            a_x,
            f"Linear Acceleration in {motion.capitalize()} Motion",
            "X Accel",
            'm/s^2'
        ),
        plot_def(
            2,
            a_y,
            f"Radial Acceleration in {motion.capitalize()} Motion",
            "Y Accel",
            'm/s^2'
        ),
        plot_def(
            3,
            omega_z,
            f"Rotational Velocity in {motion.capitalize()} Motion",
            "Z omega",
            'rad/s'
        )
    ]

    # fig, axes = plt.subplot(len(plots), 1)
    # fig.tight_layought
    for plot in plots:
        # plt.subplot(len(plots), 3, i + (plot['index'] - 1) * 3)
        plt.subplot(1, 3, i)
        plt.plot(t, plot['value'], label=plot['label'])
        # plt.plot(t, lpf(t, plot['value']), label=f"LPF for {plot['label']}")

        # multiple = 2.5
        # plt.ylim(min(plot['value']) * multiple, max(plot['value']) * multiple)

        plt.legend()
        plt.xlabel('Time [s]')
        # plt.ylabel(f"{plot['label']} [{plot['unit']}]")
        plt.title(plot['title'])
    plt.grid()
    plt.title(f"Movement for {motion} motion")
    plt.ylabel("[m/s^2] | [rad/s]")



def plot_odom(file, motion, i):
    vectors = []

    with open(file, 'r') as f:
        lines = f.readlines()
        for line in lines[1:]:
            vectors.append([float(item) for item in line.split(",")])

    every = {
        MOTION.LINE: 15,
        MOTION.CIRCLE: 10,
        MOTION.SPIRAL: 10,
    }[motion]

    x, y, u, v, th= [
        [vector[0] for vector in vectors],
        [vector[1] for vector in vectors],
        [np.cos(vector[2]) for vector in vectors],
        [np.sin(vector[2]) for vector in vectors],
        [vector[2] for vector in vectors],
    ]

    def lims(series):
        s_diff = 0.8 * (max(series) - min(series))
        s_avg = np.average(series)

        s_max = s_avg + s_diff
        s_min = s_avg - s_diff

        return [s_min, s_max]

    xlim = lims(x)
    ylim = lims(y)

    lim = np.array([min(xlim[0], ylim[0]), max(xlim[1], ylim[1])])

    plt.subplot(2, 3, i)
    plt.title(f"Plot of Odometer data for {motion} motion")
    plt.quiver(x[0:-1:every], y[0:-1:every], u[0:-1:every], v[0:-1:every])
    plt.scatter([], [], marker=r'$\longrightarrow$', c="black", s=120, label="Robot Orientation")
    plt.scatter(x[0:-1:every], y[0:-1:every], color='b', s=20, marker='x', label="Robot Position")
    plt.xlim(lim)
    plt.ylim(lim)
    plt.legend()
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.grid()

    plt.subplot(2, 3, i + 3)
    plt.title(f"Plot of Odometer data over time for {motion}")
    plt.plot(x, label='X')
    plt.plot(y, label='Y')
    plt.plot(th, label='Theta')
    plt.legend()
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('[m] | [rad]')


def plot_laser(file, motion, i):
    LINE_NUM = 1

    with open(file, 'r') as f:
        lines = f.readlines()
        angle_inc, distances, timestamp = lines[LINE_NUM].split(",")

        angle_inc = float(angle_inc)

        distances = [float(distance) for distance in distances.strip(' [] ').split(':')]

        x = [[]]
        y = [[]]

        current_angle = np.pi/2.0
        for distance in distances:
            if not math.isinf(distance):
                x[-1].append(distance * np.cos(current_angle))
                y[-1].append(distance * np.sin(current_angle))
            elif len(x[-1]) != 0:
                x.append([])
                y.append([])
            current_angle += angle_inc

        plt.figure()
        plt.title(f"Plot of range data in one timestamp")
        for x_set, y_set in zip(x, y):
            t = np.linspace(1, len(x_set), len(x_set)) * 0.05

            plt.scatter(
                x_set,
                y_set,
                color='r',
                s=1
            )
        plt.scatter([0], [0], color='b',s=80, marker='X', label='Robot Position')
        plt.scatter([], [], color='r', s=10, label='Distance Measurements')
        plt.legend()
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.grid()


    pass

def plot(sensor, motion, i):
    # Let's condition the sensors first.
    # IMU data -> Pass through
    # ODOM data -> Convert quaternion data to Euler and plot XYZ, Euler
    # LASER data -> Convert the angles into a set of points and scatter them

    in_file = FILE(sensor, motion)

    match sensor:
        case SENSOR.IMU:
            if i == 1:
                plt.figure()
            plot_imu(in_file, motion, i)
        case SENSOR.ODOM:
            plot_odom(in_file, motion, i)
        case SENSOR.LASER:
            plot_laser(in_file, motion, i)

def Main():
    os.makedirs("data_out", exist_ok=True)

    # Select which sensor you would like to plot
    sensor = SENSOR.IMU

    if sensor == SENSOR.IMU:
        # Setup A: Plot all the IMU data
        motion_list = [
            MOTION.LINE,
            MOTION.CIRCLE,
            MOTION.SPIRAL
        ]
    elif sensor == SENSOR.ODOM:
        # Setup B: Plot all the ODOM data
        motion_list = [
            MOTION.LINE,
            MOTION.CIRCLE,
            MOTION.SPIRAL
        ]
    elif sensor == SENSOR.LASER:
        # Setup C: Plot one LASER data
        motion_list = [
            MOTION.LINE
        ]

    for i, motion in enumerate(motion_list):
        plot(sensor, motion, i + 1)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    Main()