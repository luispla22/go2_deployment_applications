import os
import subprocess
import json
import threading
import random
import time
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import numpy as np
from queue import PriorityQueue
from scipy.spatial import cKDTree

# Try to import ROS 2 modules, but continue if not available
try:
    # subprocess.run(['source', '/home/unitree/unitree_ros2/cyclonedds_ws/src/install/setup.bash'], shell=True, check=False)
    subprocess.run(['source', '/home/unitree/unitree_ros2/setup.sh'], shell=True, check=False)
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import SportModeState, HeightMap, SportModeCmd
    from unitree_api.msg import Request
    from sensor_msgs.msg import PointCloud2
    import struct
    ROS_AVAILABLE = True
except ImportError:
    print("Warning: ROS 2 modules not found. Running in test mode only.")
    ROS_AVAILABLE = False

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

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
            socketio.emit('heightmap_update', json.dumps({
                'width': int(msg.width),
                'height': int(msg.height),
                'data': [min(max(float(x), 0.0), 1.0) if x < 1000000000.0 else None for x in msg.data]
            }))

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
            self.check_estop()

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
                'data': [item for sublist in self.test_heightmap for item in sublist]
            }))

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
        def heuristic(a, b):
            return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        def get_neighbors(pos):
            neighbors = []
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                new_pos = (pos[0] + dx, pos[1] + dy)
                if 0 <= new_pos[0] < len(heightmap) and 0 <= new_pos[1] < len(heightmap[0]):
                    neighbors.append(new_pos)
            return neighbors

        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                break

            for next_pos in get_neighbors(current):
                new_cost = cost_so_far[current] + heightmap[next_pos[0]][next_pos[1]]
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        return path
    
    def check_estop(self):

        # Define the front area of the robot
        front_area = np.array([[-0.2, 0.2], [0.3, 0.5]])  # [x_min, x_max], [y_min, y_max]

        # Filter points in the front area
        mask = (self.gridded_points[:, 0] >= front_area[0, 0]) & (self.gridded_points[:, 0] <= front_area[0, 1]) & \
               (self.gridded_points[:, 1] >= front_area[1, 0]) & (self.gridded_points[:, 1] <= front_area[1, 1])
        front_points = self.gridded_points[mask]

        estop_condition = False

        if len(front_points) > 0:
            # Check if any point is too high (higher than 20cm below the robot)
            if np.any(front_points[:, 2] > -0.2):
                estop_condition = True

                # Stop the robot
                self.velocity_command = Request()
                self.velocity_command.header.identity.api_id = 1003
                self.velocity_publisher.publish(self.velocity_command)


                # Emit a message if transitioning from non-estop to estop state
                if not self.is_estopped:
                    socketio.emit('estop_status', {'is_estopped': True})
                self.is_estopped = True
                return

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

        # build points

        # x_init, y_init, z_init = self.current_position
        # yaw_init = self.current_yaw
        # vx_local, vy_local, vyaw = 0.5, 0.0, 0.0
        # dt = 0.2

        # path = []

        # for i in range(300):
        #     time = i * dt
        #     vx_global = vx_local * np.cos(yaw_init) - vy_local * np.sin(yaw_init)
        #     vy_global = vx_local * np.sin(yaw_init) + vy_local * np.cos(yaw_init)
        #     path_point = {
        #         "t_from_start": time,
        #         "x": x_init + time * vx_global,
        #         "y": y_init + time * vy_global,
        #         "yaw": yaw_init + time * vyaw,
        #         "vx": vx_global,
        #         "vy": vy_global,
        #         "vyaw": vyaw,
        #     } 
        #     path.append(path_point)

        # dt = 0.2
        # time_seg = 0.2

        # t = 0
        # for k in range(3000):
        #     t += dt
        #     ts = t - time_seg
        #     for i in range(30):
        #         ts += time_seg
        #         px_local = 0.5 * np.sin(0.5 * ts)
        #         py_local = 0
        #         vx_local = 0.5 * np.cos(0.5 * ts)
        #         vy_local = 0

        #         px_global = x_init + px_local * np.cos(yaw_init) - py_local * np.sin(yaw_init)
        #         py_global = y_init + px_local * np.sin(yaw_init) + py_local * np.cos(yaw_init)
                
        #         vx_global = vx_local * np.cos(yaw_init) - vy_local * np.sin(yaw_init)
        #         vy_global = vx_local * np.sin(yaw_init) + vy_local * np.cos(yaw_init)

        #         path_point = {
        #             "t_from_start": i * time_seg,
        #             "x": px_global,
        #             "y": py_global,
        #             "yaw": yaw_init + ts * vyaw,
        #             "vx": vx_global,
        #             "vy": vy_global,
        #             "vyaw": vyaw,
        #         } 
        #         path.append(path_point)

        #     self.velocity_command.parameter = json.dumps(path)
        #     self.velocity_command.header.identity.api_id = 1018

        #     self.velocity_publisher.publish(self.velocity_command)
        #     self.log(f"published path command {t}")

        #     time.sleep(dt)
    
    def execute_path(self, path):
        self.executing_path = True
        self.current_path = path
        
        for point in self.current_path:
            if not self.executing_path:
                break
            
            # Calculate velocity commands based on current position and next point
            velocity_x = 0.5
            velocity_y = 0.0
            
            # Create velocity command
            if ROS_AVAILABLE:
                self.velocity_command = Request()
                self.velocity_command.parameter = json.dumps({"x": velocity_x, "y": velocity_y, "z": 0.0})
                self.velocity_command.header.identity.api_id = 1008
                # self.velocity_command = SportModeState()
                # self.velocity_command.velocity = [velocity_x, velocity_y, 0.0]
                # self.velocity_command.yaw_speed = 0.0
            
            # Check for e-stop condition
            self.check_estop()
            
            if self.is_estopped:
                self.velocity_command.velocity = [0.0, 0.0, 0.0]
                self.velocity_command.yaw_speed = 0.0
                self.log("E-Stop activated! Robot stopped.", "warning")
            
            if ROS_AVAILABLE:
                self.velocity_publisher.publish(self.velocity_command)
            else:
                self.log(f"Test mode: Publishing velocity command: x={self.velocity_command.velocity[0]}, y={self.velocity_command.velocity[1]}")
            
            print("PUBLISH VELOCITY COMMAND", self.velocity_command)
            
            time.sleep(0.1)
        
        self.executing_path = False
        self.current_path = None
        self.log("Path execution completed")

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
    heightmap = np.array(robot_dog_node.test_heightmap)  # Use the test heightmap for now
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