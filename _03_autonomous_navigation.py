import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import math

from utils.teleop_visualizer import TeleopVisualizer
from utils.go2_test_interface import Go2Interface
import time



from collections import deque

def plan_path_bfs(lidar_data, destination, search_map_dim=64):
    # Define the movement directions (up, down, left, right)
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    # downsize the LIDAR map and destination to search_map_dim
    original_rows, original_cols = lidar_data.shape
    destination = (int(destination[1] * search_map_dim / original_rows), int(destination[0] * search_map_dim / original_cols))
    lidar_data = cv2.resize(lidar_data, (search_map_dim, search_map_dim))

    # Get the dimensions of the LIDAR data
    rows, cols = lidar_data.shape

    # Define the start position as the center of the LIDAR data
    start = (rows // 2, cols // 2)

    # Create a queue to store the nodes to visit
    queue = deque()
    queue.append((start, [start]))

    # Create a set to keep track of visited nodes
    visited = set()

    while queue:
        current_pos, path = queue.popleft()

        # Check if the current position is the destination
        if current_pos == destination:
            # revert downsizing operation
            path = [(int(p[0] * original_rows / search_map_dim), int(p[1] * original_cols / search_map_dim)) for p in path]
            return path

        # Mark the current position as visited
        visited.add(current_pos)

        # Explore the neighboring positions
        for direction in directions:
            new_pos = (current_pos[0] + direction[0], current_pos[1] + direction[1])

            # Check if the new position is within the LIDAR data bounds
            if 0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols:
                # Check if the new position is not an obstacle and hasn't been visited
                if lidar_data[new_pos[0], new_pos[1]] != 0 and new_pos not in visited:
                    new_path = path + [new_pos]
                    queue.append((new_pos, new_path))
                    visited.add(new_pos)

    # If no path is found, return an empty list
    return []




def safety_check_fails(lidar_data):
    if lidar_data[0] < 0.1:
        return True
    return False

def main():

    teleop_visualizer = TeleopVisualizer()
    go2_interface = Go2Interface()

    for i in range(10000):
        rgb_image = go2_interface.get_rgb_image()
        lidar_data = go2_interface.get_lidar_array()

        robot_vel = go2_interface.get_robot_vel_xy()

        teleop_visualizer.set_rgb_image(rgb_image)
        teleop_visualizer.set_lidar_data(lidar_data)

        teleop_visualizer.step()

        if safety_check_fails(lidar_data) and robot_vel > 0.0:
            # trigger emergency stop!
            print("Emergency stop!")
            go2_interface.set_robot_vel(x=0.0, y=0.0)

        time.sleep(0.01)


if __name__ == "__main__":
    main()