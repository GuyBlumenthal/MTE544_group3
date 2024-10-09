import os
from filePlotter import plot_errors
from utilities import Logger
import numpy as np
import matplotlib.pyplot as plt
from math import floor

class SENSOR:
    IMU = "imu"
    ODOM = "odom"
    LASER = "laser"

class MOTION:
    LINE = "line"
    CIRCLE = "circle"
    SPIRAL = "spiral"

FILE = lambda sensor, motion: f"lab1_data/{sensor}_content_{motion}.csv"

def plot_imu(file):
    pass

def plot_odom(file):
    vectors = []
    
    with open(file, 'r') as f:
        lines = f.readlines()
        for line in lines[1:]:
            vectors.append([float(item) for item in line.split(",")])
            
    x, y, u, v = [
        [vector[0] for vector in vectors],
        [vector[1] for vector in vectors],
        [np.cos(vector[2]) for vector in vectors],
        [np.sin(vector[2]) for vector in vectors],
    ]

    xlim = [min(x), max(x)]
    ylim = [min(y), max(y)]

    plt.quiver(x, y, u, v)
    plt.xlim(xlim[0], xlim[1])
    plt.ylim(ylim[0], ylim[1])
    plt.grid()
    plt.show()
    

def plot_laser(file):
    pass

def plot(sensor, motion):
    # Let's condition the sensors first.
    # IMU data -> Pass through
    # ODOM data -> Convert quaternion data to Euler and plot XYZ, Euler
    # LASER data -> Convert the angles into a set of points and scatter them

    in_file = FILE(sensor, motion)

    match sensor:
        case SENSOR.IMU:
            plot_imu(in_file)
        case SENSOR.ODOM:
            plot_odom(in_file)
        case SENSOR.LASER:
            plot_laser(in_file)

def Main():
    os.makedirs("data_out", exist_ok=True)

    # Set up permutation table. Comment out lines for sensors and motions to exclude
    sensor_list = [
        # SENSOR.IMU,
        SENSOR.ODOM,
        # SENSOR.LASER
    ]

    motion_list = [
        # MOTION.LINE,
        MOTION.CIRCLE,
        # MOTION.SPIRAL
    ]

    for sensor in sensor_list:
        for motion in motion_list:
            plot(sensor, motion)

if __name__ == "__main__":
    Main()