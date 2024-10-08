from filePlotter import plot_errors
from utilities import Logger

class SENSOR:
    IMU = "imu"
    ODOM = "odom"
    LASER = "laser"

class MOTION:
    LINE = "line"
    CIRCLE = "circle"
    SPIRAL = "spiral"

FILE = lambda sensor, motion: f"lab1_data/{sensor}_content_{motion}.csv"

def condition_odom(file):
    return_file = '__pycache/__odom_data.csv'
    Logger(return_file, headers=['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'stamp'])

    

def condition_laser(file):
    print("Then")

def plot(sensor, motion):
    # Let's condition the sensors first.
    # IMU data -> Pass through
    # ODOM data -> Convert quaternion data to Euler and plot XYZ, Euler
    # LASER data -> Convert the angles into a set of points and scatter them

    in_file = FILE(sensor, motion)
    out_file = in_file

    match sensor:
        case SENSOR.ODOM:
            out_file = condition_odom(in_file)
        case SENSOR.LASER:
            out_file = condition_laser(in_file)

    # plot_errors(out_file)

def Main():
    # Set up permutation table. Comment out lines for sensors and motions to exclude
    sensor_list = [
        SENSOR.IMU,
        SENSOR.ODOM,
        SENSOR.LASER
    ]

    motion_list = [
        MOTION.LINE,
        MOTION.CIRCLE,
        MOTION.SPIRAL
    ]

    for sensor in sensor_list:
        for motion in motion_list:
            plot(sensor, motion)

if __name__ == "__main__":
    Main()