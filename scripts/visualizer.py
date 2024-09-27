import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import sys

def parse_heuristic(filename, x_size, y_size):
    heuristic_raw = np.loadtxt(filename, delimiter=',')

    # Getting next largest value and setting int_max values in heuristic raw to that * a factor.
    heuristic_masked = heuristic_raw.copy()
    heuristic_masked = np.ma.masked_equal(heuristic_raw, 2147483647.0)
    next_largest = heuristic_masked.max()
    
    heuristic_raw[heuristic_raw == 2147483647.0] = next_largest * 1.5
    heuristic = heuristic_raw.reshape((y_size, x_size))
    return heuristic

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'C', "Expected 'C' in the third line"
        collision_threshold = int(file.readline().strip())

        assert file.readline().strip() == 'R', "Expected 'R' in the fifth line"
        robotX, robotY = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'T', "Expected 'T' in the seventh line"
        target_trajectory = []
        line = file.readline().strip()
        while line != 'M':
            x, y = map(float, line.split(','))
            target_trajectory.append({'x': x, 'y': y})
            line = file.readline().strip()

        costmap = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap.append(row)

        costmap = np.asarray(costmap).T

    return x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap

def parse_robot_trajectory_file(filename):
    robot_trajectory = []
    with open(filename, 'r') as file:
        for line in file:
            t, x, y = map(int, line.strip().split(','))
            robot_trajectory.append({'t': t, 'x': x, 'y': y})

    return robot_trajectory

SPEEDUP = 500000

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)

    x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap = parse_mapfile(sys.argv[1])
    # heuristic = parse_heuristic("../heuristic", x_size, y_size)
    robot_trajectory = parse_robot_trajectory_file('../output/robot_trajectory.txt')

    fig, ax1 = plt.subplots(1)

    ax1.imshow(costmap)
    ax1.set_title('Map 5')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    traj_x = [p['x'] for p in robot_trajectory]
    traj_y = [p['y'] for p in robot_trajectory]
    obj_x = [p['x'] for p in target_trajectory]
    obj_y = [p['y'] for p in target_trajectory]
    
    ax1.plot(traj_x, traj_y, 'b', marker='o', lw=2, label='Robot Trajectory')
    ax1.plot(obj_x, obj_y, 'r', marker='o', lw=2, label='Target Trajectory', alpha=0.5)
    ax1.legend()

    # line1, = ax1.plot([], [], lw=2, marker='o', color='b', label='robot')
    # line2, = ax1.plot([], [], lw=2, marker='o', color='r', label='target')

    # def init():
    #     line1.set_data([], [])
    #     line2.set_data([], [])
    #     return line1, line2

    # def update(frame):
    #     line1.set_data([p['x'] for p in robot_trajectory[:frame+1]], [p['y'] for p in robot_trajectory[:frame+1]])

    #     t = robot_trajectory[frame+1]['t']
    #     line2.set_data([p['x'] for p in target_trajectory[:t]], [p['y'] for p in target_trajectory[:t]])

    #     plt.pause((robot_trajectory[frame+1]['t']-robot_trajectory[frame]['t'])/SPEEDUP)

    #     return line1, line2

    # ani = FuncAnimation(fig, update, frames=len(robot_trajectory)-1, init_func=init, blit=False, interval=1)

    plt.legend()
    plt.show()
