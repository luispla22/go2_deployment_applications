import os
import sys
import subprocess
import json
import threading
import random
import time
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import logging
import numpy as np
from collections import deque
 
# Try to import ROS 2 modules, but continue if not available
try:
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import SportModeState, HeightMap, SportModeCmd
    from unitree_api.msg import Request
    from sensor_msgs.msg import PointCloud2
    import struct
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ROS 2 modules not found. Running in test mode only.")
    RED = '\033[91m'
    RESET = '\033[0m'

    print(f"{RED}If you are on the dog, don't forget to run `source unitree_ros2/setup.sh` or similar before running the script, or add this to your ~/.bashrc file!{RESET}")

    ROS_AVAILABLE = False

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')
app.logger.setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)

class RobotDogNode:
    def __init__(self):
        self.testing_mode = not ROS_AVAILABLE
        self.point_cloud_buffer = []
        self.test_heightmap = self.generate_test_heightmap()
        self.executing_path = False
        self.current_path = None
        self.current_position = np.zeros(3)  # Initialize current position
        self.current_yaw = 0.0  # Initialize current yaw
        
        self.estop_active = False
        self.is_estopped = False
        self.velocity_command = None
        self.gridded_points = None

        if ROS_AVAILABLE:
            rclpy.init()
            self.node = rclpy.create_node('robot_dog_node')
            self.node.create_subscription(SportModeState, '/sportmodestate', self.sportmode_callback, 10)
            self.node.create_subscription(HeightMap, '/utlidar/height_map_array', self.heightmap_callback, 10)
            self.node.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.pointcloud_callback, 10)
            self.velocity_publisher = self.node.create_publisher(Request, '/api/sport/request', 10)
            self.log("ROS initialized successfully")
        else:
            self.log("ROS not available. Running in testing mode only.", "warning")
        
        self.log("Robot Dog Node initialized")

    def log(self, message, level="info"):
        print(f"[{level.upper()}] {message}")  # Print to server console
        socketio.emit('log', {'message': message, 'level': level})

    def sportmode_callback(self, msg):
        if not self.testing_mode:
            self.current_position = np.array(msg.position)  # Update current position
            self.current_yaw = msg.imu_state.rpy[2]  # Update current yaw
            socketio.emit('sportmode_update', json.dumps({
                'velocity_x': float(msg.velocity[0]),
                'velocity_y': float(msg.velocity[1]),
                'yaw_speed': float(msg.yaw_speed),
                'position_x': float(msg.position[0]),
                'position_y': float(msg.position[1]),
                'position_z': float(msg.position[2]),
                'yaw': float(msg.imu_state.rpy[2]),
            }))

    def heightmap_callback(self, msg):
        if not self.testing_mode:
            # Extract relevant information from the message
            width = int(msg.width)
            height = int(msg.height)
            self.heightmap_origin_x, self.heightmap_origin_y = float(msg.origin[0]), float(msg.origin[1])
            self.heightmap_resolution = float(msg.resolution)
            
            # Convert heightmap data to numpy array
            heightmap = np.array(msg.data).reshape(height, width)

            # Calculate robot position in grid coordinates
            robot_x = int((self.current_position[0] - self.heightmap_origin_x) / self.heightmap_resolution)
            robot_y = int((self.current_position[1] - self.heightmap_origin_y) / self.heightmap_resolution)

            # Ensure robot position is within the heightmap bounds
            robot_x = np.clip(robot_x, 0, width - 1)
            robot_y = np.clip(robot_y, 0, height - 1)

            # Create rotation matrix
            angle = self.current_yaw - np.pi/2  # Negative because we want to rotate the map, not the robot
            cos_angle, sin_angle = np.cos(angle), np.sin(angle)
            rotation_matrix = np.array([[cos_angle, -sin_angle],
                                        [sin_angle, cos_angle]])

            # Create grid of coordinates
            y, x = np.indices((height, width))
            coordinates = np.stack((x.ravel() - robot_x, y.ravel() - robot_y), axis=-1)

            # Apply rotation
            rotated_coordinates = np.dot(coordinates, rotation_matrix.T)
            rotated_x, rotated_y = rotated_coordinates[:, 0], rotated_coordinates[:, 1]

            # Reshape back to grid
            rotated_x = rotated_x.reshape(height, width) + robot_x
            rotated_y = rotated_y.reshape(height, width) + robot_y

            # Interpolate values (nearest neighbor)
            valid_indices = (rotated_x >= 0) & (rotated_x < width) & (rotated_y >= 0) & (rotated_y < height)
            rotated_heightmap = np.full((height, width), 1000000000.0)  # Fill with "unknown" value
            rotated_heightmap[valid_indices] = heightmap[rotated_y[valid_indices].astype(int), rotated_x[valid_indices].astype(int)]

            # Reflect along x and y
            self.rotated_heightmap = rotated_heightmap[::-1, :]

            # Threshold by obstacle height
            obstacle_threshold = 0.25
            self.rotated_heightmap[self.rotated_heightmap > 1.0] = 0
            self.rotated_heightmap = self.rotated_heightmap > obstacle_threshold

            # Dilate
            dilated_heightmap = self.dilate_obstacles(self.rotated_heightmap, radius=4)
            self.rotated_heightmap[dilated_heightmap > 0] = 0.5

            # Add a colored dot for the robot (using a distinctive value, e.g., 2.0)
            emitted_heightmap = self.rotated_heightmap[:, :]
            robot_marker_value = 2.0
            emitted_heightmap[robot_y, robot_x] = robot_marker_value

            # Flatten the heightmap and convert to a list of Python floats
            processed_data = emitted_heightmap.flatten().tolist()

            # Prepare the data for emission
            emitted_data = [
                min(max(float(x), 0.0), 1.0) if x < 1000000000.0 else None if x != robot_marker_value else robot_marker_value
                for x in processed_data
            ]

            # Ensure all data is JSON serializable
            json_data = {
                'width': width,
                'height': height,
                'data': emitted_data,
                'robot_position': [int(robot_x), int(robot_y)],
                'rotation': float(np.degrees(self.current_yaw))
            }

            socketio.emit('heightmap_update', json.dumps(json_data))
            
    def pointcloud_callback(self, msg):
        if not self.testing_mode:
            # Unpack points from the message
            points = np.array([[struct.unpack_from('fff', msg.data, offset=i)[j] for j in range(3)] 
                            for i in range(0, len(msg.data), msg.point_step * 4)])  # 4x downsampling
            
            # Translate points by current position
            points -= self.current_position
            
            # Rotate points by yaw
            cos_yaw = np.cos(-self.current_yaw)
            sin_yaw = np.sin(-self.current_yaw)
            rotation_matrix = np.array([
                [cos_yaw, -sin_yaw, 0],
                [sin_yaw, cos_yaw, 0],
                [0, 0, 1]
            ])
            points = np.dot(points, rotation_matrix.T)
            
            points[:, 1] = -points[:, 1]
            
            # Update point cloud buffer
            self.point_cloud_buffer.append(points)
            if len(self.point_cloud_buffer) > 15:
                self.point_cloud_buffer.pop(0)
            
            # Aggregate points across all timesteps
            all_points = np.concatenate(self.point_cloud_buffer, axis=0)
            
            # Define grid parameters
            grid_size = 3.0  # 3m x 3m grid
            cell_size = 0.1  # 10cm resolution
            grid_shape = 30  # 30x30 grid
            
            # Create empty grid for max heights
            grid = np.full((grid_shape, grid_shape), -np.inf)
            
            # Compute cell indices for each point
            cell_indices = np.floor((all_points[:, :2] + grid_size/2) / cell_size).astype(int)
            
            # Filter out points outside the grid
            mask = np.all((cell_indices >= 0) & (cell_indices < grid_shape), axis=1)
            cell_indices = cell_indices[mask]
            all_points = all_points[mask]
            
            # Compute max height for each cell
            for idx, point in zip(cell_indices, all_points):
                grid[idx[0], idx[1]] = max(grid[idx[0], idx[1]], point[2])
            
            # Create grid cell centers
            x_centers = np.linspace(cell_size/2 - grid_size/2, grid_size/2 - cell_size/2, grid_shape)
            y_centers = np.linspace(cell_size/2 - grid_size/2, grid_size/2 - cell_size/2, grid_shape)
            xx, yy = np.meshgrid(x_centers, y_centers)
            
            # Flatten grid and create mask for non-empty cells
            grid_flat = grid.ravel()
            non_empty_mask = ~np.isinf(grid_flat)
            
            # Create 3D points at grid cell centers with max heights, excluding empty cells
            gridded_points = np.column_stack((xx.ravel()[non_empty_mask], 
                                            yy.ravel()[non_empty_mask], 
                                            grid_flat[non_empty_mask]))

            # Store the gridded points for e-stop checking
            self.gridded_points = gridded_points
            
            # Convert to list for JSON serialization
            gridded_points_list = gridded_points.tolist()
            
            # Check estop and emit status
            self.check_estop(gridded_points)

            # Emit to socketio
            socketio.emit('pointcloud_update', json.dumps({'points': gridded_points_list}))

            return gridded_points_list

    def generate_test_heightmap(self, width=50, height=50):
        # Create a base grid filled with high values (walls)
        grid = np.ones((height, width))

        # Create horizontal hallway
        h_start = int(0.4 * height)
        h_end = int(0.6 * height)
        grid[h_start:h_end, :] = 0

        # Create vertical hallway
        v_start = int(0.4 * width)
        v_end = int(0.6 * width)
        grid[:, v_start:v_end] = 0

        # Clip values to be between 0 and 1
        grid = np.clip(grid, 0, 1)

        return grid.tolist()

    def generate_fake_data(self):
        self.log("Generating fake data")
        while self.testing_mode:
            socketio.emit('sportmode_update', json.dumps({
                'velocity_x': random.uniform(-0.05, 0.05),
                'velocity_y': random.uniform(-0.05, 0.05),
                'yaw_speed': random.uniform(-0.05, 0.05),
            }))

            socketio.emit('heightmap_update', json.dumps({
                'width': len(self.test_heightmap[0]),
                'height': len(self.test_heightmap),
                'data': [item for sublist in self.test_heightmap for item in sublist],
                'robot_position': [25, 25],
                'rotation': 0.0
            }))
            
            # Store the data in the RobotDogNode class
            self.rotated_heightmap = np.array(self.test_heightmap)

            points = []
            for y in range(len(self.test_heightmap)):
                for x in range(len(self.test_heightmap[0])):
                    if random.random() < 0.1:
                        height = self.test_heightmap[y][x]
                        points.append([
                            (x / len(self.test_heightmap[0]) - 0.5),
                            (y / len(self.test_heightmap) - 0.5),
                            height * 0.5
                        ])
            socketio.emit('pointcloud_update', json.dumps({'points': points}))

            time.sleep(0.1)
        self.log("Stopped generating fake data")
        
    def spin(self):
        if ROS_AVAILABLE:
            self.log("Starting ROS spin")
            rclpy.spin(self.node)
        else:
            self.log("Starting fake data generation")
            self.generate_fake_data()
            
    def calculate_path(self, start, goal, heightmap):
        '''
        Breadth-first search for a path from `start` to `goal`
        '''
        
        height, width = heightmap.shape

        def is_valid(pos):
            # query pos is in bounds and not in collision
            return (0 <= pos[0] < height and 
                    0 <= pos[1] < width and 
                    heightmap[pos[0]][pos[1]] == 0)

        def get_neighbors(pos):
            # get valid neighbors including diagonals
            neighbors = []
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                new_pos = (pos[0] + dx, pos[1] + dy)
                if is_valid(new_pos):
                    neighbors.append(new_pos)
            return neighbors

        queue = deque([start])
        visited = set([start])
        came_from = {start: None}

        while queue: # Main BFS Loop
            current = queue.popleft()
            
            if current == goal:
                break

            for next_pos in get_neighbors(current):
                if next_pos not in visited:
                    queue.append(next_pos)
                    visited.add(next_pos)
                    came_from[next_pos] = current

        # Reconstruct path
        if goal not in came_from:
            print("No path found")
            return []

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        return path

    def dilate_obstacles(self, heightmap, radius):
        # Create a circular kernel
        y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
        kernel = x**2 + y**2 <= radius**2

        # Perform dilation
        dilated = np.zeros_like(heightmap)
        for i in range(-radius, radius+1):
            for j in range(-radius, radius+1):
                if kernel[i+radius, j+radius]:
                    dilated |= np.roll(np.roll(heightmap, i, axis=0), j, axis=1)

        return dilated   
    
    def check_estop(self, gridded_points):

        # Define the front area of the robot
        front_area = np.array([[-0.2, 0.2], [0.3, 0.5]])  # [x_min, x_max], [y_min, y_max]

        # Filter points in the front area
        mask = (gridded_points[:, 0] >= front_area[0, 0]) & (gridded_points[:, 0] <= front_area[0, 1]) & \
               (gridded_points[:, 1] >= front_area[1, 0]) & (gridded_points[:, 1] <= front_area[1, 1])
        front_points = self.gridded_points[mask]

        estop_condition = False

        if len(front_points) > 0:
            # Check if any point is too high (higher than 20cm below the robot)
            if np.any(front_points[:, 2] > -0.2):
                estop_condition = True

                # Stop the robot
                self.stop_robot()

        # Check if the estop status has just changed
        if estop_condition != self.is_estopped:
            # Commnicate the new state to the web dashboard
            socketio.emit('estop_status', {'is_estopped': estop_condition})
        self.is_estopped = estop_condition
    
    def step_forward(self):

        # Command forward
        self.velocity_command = Request()
        self.velocity_command.parameter = json.dumps({"x": 0.5, "y": 0.0, "z": 0.0})
        self.velocity_command.header.identity.api_id = 1008
        self.log("Commanded forward")

        # wait one secong
        ts = time.time()
        while time.time() - ts < 3.0:
            self.velocity_publisher.publish(self.velocity_command)
            time.sleep(0.1)

        # command stop
        self.velocity_command = Request()
        self.velocity_command.header.identity.api_id = 1003
        self.velocity_publisher.publish(self.velocity_command)
        self.log("Commanded stop")
    
    def execute_path(self, path):
        self.executing_path = True
        self.current_path = path

        ## Convert the path to the robot's local frame
        # Scale
        path = [(p[0] * self.heightmap_resolution, p[1] * self.heightmap_resolution) for p in path]
        # Shift
        start_point = path[0]
        path = [(p[0] - start_point[0], p[1] - start_point[1]) for p in path]
        path = [(p[0], -1 * p[1]) for p in path]

        # Execute the path open-loop
        for i in range(len(path) - 1):
            if not self.executing_path:
                break
            
            target = np.array(path[i + 1])
            prev_target = np.array(path[i])
            velocity = target - prev_target
            speed = 0.3
            duration = np.linalg.norm(velocity) / speed
            velocity = velocity / duration
            print(target, self.current_position[:2], velocity, duration)

            t_start = time.time()
            
            while self.executing_path:
                
                if time.time() - t_start > duration:  # If within 10cm of target, move to next point
                    break
                
                velocity_x, velocity_y = velocity[0], velocity[1]

                # Create and send velocity command
                self.velocity_command = Request()
                self.velocity_command.parameter = json.dumps({"x": float(velocity_x), "y": float(velocity_y), "z": 0.0})
                self.velocity_command.header.identity.api_id = 1008
                
                # Check for e-stop condition
                self.check_estop(self.gridded_points)
                
                if self.is_estopped:
                    self.stop_robot()
                    self.log("E-Stop activated! Robot stopped.", "warning")
                    return
                
                if ROS_AVAILABLE:
                    self.velocity_publisher.publish(self.velocity_command)
                else:
                    self.log(f"Test mode: Publishing velocity command: x={velocity_x:.2f}, y={velocity_y:.2f}")
                
                time.sleep(0.02)  # Send command every 50ms
            
        # Stop the robot at the end of the path
        self.stop_robot()
        self.log("Path execution completed")
        self.executing_path = False
        self.current_path = None

    def stop_robot(self):
        self.velocity_command = Request()
        self.velocity_command.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
        self.velocity_command.header.identity.api_id = 1003  # Use api_id 1003 for stop command
        if ROS_AVAILABLE:
            self.velocity_publisher.publish(self.velocity_command)
        else:
            self.log("Test mode: Stopping robot")

robot_dog_node = None

@socketio.on('toggle_testing_mode')
def handle_testing_mode(data):
    global robot_dog_node
    if ROS_AVAILABLE:
        robot_dog_node.testing_mode = data['testing_mode']
        if robot_dog_node.testing_mode:
            robot_dog_node.log("Switched to testing mode")
            threading.Thread(target=robot_dog_node.generate_fake_data, daemon=True).start()
        else:
            robot_dog_node.log("Switched to real data mode")
    else:
        robot_dog_node.log("ROS is not available. Always running in test mode.", "warning")

@app.route('/')
def index():
    with open('dbdash_dashboard.html', 'r') as file:
        template_content = file.read()
    return render_template_string(template_content, ROS_AVAILABLE=ROS_AVAILABLE)

@socketio.on('plan_path')
def handle_plan_path(data):
    global robot_dog_node
    start = tuple(data['start'])
    end = tuple(data['end'])
    heightmap = robot_dog_node.rotated_heightmap[::-1, :]
    path = robot_dog_node.calculate_path(start, end, heightmap)
    print(start, end, path)
    socketio.emit('path_result', json.dumps(path))
    
@socketio.on('execute_path')
def handle_execute_path(data):
    global robot_dog_node
    path = data['path']
    robot_dog_node.log("Received command to execute path")
    threading.Thread(target=robot_dog_node.execute_path, args=(path,), daemon=True).start()
    
@socketio.on('step_forward')
def handle_step_forward(data):
    global robot_dog_node
    threading.Thread(target=robot_dog_node.step_forward, args=(), daemon=True).start()

if __name__ == '__main__':
    robot_dog_node = RobotDogNode()
    threading.Thread(target=robot_dog_node.spin, daemon=True).start()
    
    @socketio.on('connect')
    def handle_connect():
        if not ROS_AVAILABLE:
            robot_dog_node.log("Connected to dashboard. Running in testing mode only.", "warning")
        else:
            robot_dog_node.log("Connected to dashboard. ROS 2 is available.")
    
    socketio.run(app, host='0.0.0.0', port=9000, debug=True)