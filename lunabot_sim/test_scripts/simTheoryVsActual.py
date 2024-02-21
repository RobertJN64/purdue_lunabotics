import math
import csv
import numpy as np

csv_file_name = "csvs/mpc_friction0.8_0.5_angular1.csv"

import matplotlib.pyplot as plt

def plot_trajectory(points, real_poses):
    """
    Plot the trajectory of 2D points.

    Parameters:
    - points (list of tuples): List of 2D points as tuples (x, y).
    """
    x_values, y_values, _ = zip(*points)

    plt.plot(x_values, y_values, marker='o', markersize=2, linestyle='-', color='b')
    x_values2, y_values2, _ = zip(*real_poses)
    plt.plot(x_values2, y_values2, marker='o', markersize=2, linestyle='-', color='r')
    plt.title('Trajectory Plot')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')

    plt.axis('equal')

    plt.grid(True)
    plt.show()

def calc_new_pos(prev_pos, time, prev_time, prev_lin_vel, prev_ang_vel):
    """
    Calculate the new position of the robot.

    Parameters:
    - prev_pos (list): The previous position of the robot [x, y, theta].
    - time (float): The current time.
    - prev_time (float): The previous time.
    - prev_lin_vel (float): The previous linear velocity.
    - prev_ang_vel (float): The previous angular velocity.

    Returns:
    - list: The new position of the robot [x, y, theta].
    """
    dt = time - prev_time
    x, y, theta = prev_pos
    x += prev_lin_vel * math.cos(theta) * dt
    y += prev_lin_vel * math.sin(theta) * dt
    theta += prev_ang_vel * dt
    return [x, y, theta]


with open(csv_file_name) as fd:
    spamreader = csv.reader(fd)

    first_row = True

    prev_time, prev_lin_vel, prev_ang_vel = 0, 0, 0

    count = 0
    
    poses = []
    real_poses = []

    for row in spamreader:
        count += 1
        floatArr = [float(x) for x in row]
        time, x, y, theta, lin_vel, ang_vel = floatArr
        time = time / 1000000000
 
        if not first_row:
            pos = calc_new_pos(pos, time, prev_time, prev_lin_vel, prev_ang_vel)
        else:
            first_row = False
            pos = [x, y, theta]

        poses.append(pos)
        real_poses.append([x, y, theta])
        prev_time = time
        prev_lin_vel = lin_vel
        prev_ang_vel = ang_vel


    plot_trajectory(poses, real_poses)