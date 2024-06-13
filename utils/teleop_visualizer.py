import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2
import tkinter as tk
import tkinter.messagebox as messagebox
from PIL import Image, ImageTk
import math

class TeleopVisualizer:
    def __init__(self):

        self.timestep = 0

        # initialize sensor data with blank values before first timestep
        self.rgb_image = np.zeros([128, 128], dtype=np.uint8)
        self.lidar_image = np.zeros([128, 128], dtype=np.uint8)

        self.initialize_tk_window()
        self.step()

    def step(self):
        # Update the Tkinter window
        self.window.update()

    def set_rgb_image(self, rgb_image):
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        rgb_image = cv2.resize(rgb_image, (400, 400))  # Adjust the size as needed
        self.rgb_image = (rgb_image * 255).astype(np.uint8)
        
        photo_rgb = ImageTk.PhotoImage(Image.fromarray(self.rgb_image))
        self.following_label_rgb.configure(image=photo_rgb)
        self.following_label_rgb.image = photo_rgb

    def set_lidar_data(self, lidar_data):
        lidar_image = cv2.cvtColor(lidar_data, cv2.COLOR_BGR2RGB)
        lidar_image = cv2.resize(lidar_image, (400, 400))  # Adjust the size as needed
        self.lidar_image = (lidar_image * 255).astype(np.uint8)

        lidar_photo = ImageTk.PhotoImage(Image.fromarray(self.lidar_image))
        self.following_label_depth.configure(image=lidar_photo)
        self.following_label_depth.image = lidar_photo

    # Function to move the robot forward
    def move_forward(self, event=None):
        pass

    # Function to move the robot backward
    def move_backward(self, event=None):
        pass

    # Function to turn the robot left
    def turn_left(self, event=None):
        pass

    # Function to turn the robot right
    def turn_right(self, event=None):
        pass
        
    def await_user_click(self):
        def on_click(event):
            x = event.x
            y = event.y
            messagebox.showinfo("Click Coordinates", f"Clicked at ({x}, {y})")
            self.clicked_coordinates = (x, y)
            self.window.quit()

        self.following_label_depth.bind("<Button-1>", on_click)
        self.window.mainloop()
        self.following_label_depth.unbind("<Button-1>")
        return self.clicked_coordinates
    
    def display_popup(self, popup_text):
        messagebox.showinfo("Map Information", popup_text)

    def display_path(self, path):
        # Create a copy of the overhead camera image
        image = self.following_rgb_image.copy()

        # Convert the path coordinates to image coordinates
        image_path = [(col, row) for row, col in path]

        # Draw the path on the image
        for i in range(len(image_path) - 1):
            cv2.line(image, image_path[i], image_path[i + 1], (0, 255, 0), 2)

        # Convert the image to a PhotoImage object
        photo = ImageTk.PhotoImage(Image.fromarray(image))

        # Update the overhead camera label with the path
        self.following_label_rgb.configure(image=photo)
        self.following_label_rgb.image = photo

    def execute_path(self, path, meters_per_pixel=0.02):
        if len(path) == 0:
            return

        # Set the desired speed of the robot (in meters per second)
        speed = 1.5

        path = [[-p[1], -p[0]] for p in path]

        # Calculate the total distance of the path in meters
        total_distance = 0
        for i in range(len(path) - 1):
            dx = (path[i + 1][1] - path[i][1]) * meters_per_pixel
            dy = (path[i + 1][0] - path[i][0]) * meters_per_pixel
            total_distance += math.sqrt(dx ** 2 + dy ** 2)

        # Calculate the total time to complete the path
        total_time = total_distance / speed

        # Calculate the number of waypoints based on the desired time step
        time_step = 0.1  # Adjust this value to change the spacing between waypoints
        num_waypoints = int(total_time / time_step)

        # Generate equally spaced waypoints along the path using interpolation
        waypoints = []
        for i in range(num_waypoints + 1):
            t = i * time_step / total_time
            index = int(t * (len(path) - 1))
            fraction = t * (len(path) - 1) - index
            if index < len(path) - 1:
                waypoint = (
                    path[index][0] + fraction * (path[index + 1][0] - path[index][0]),
                    path[index][1] + fraction * (path[index + 1][1] - path[index][1])
                )
            else:
                waypoint = path[-1]
            waypoints.append(waypoint)

        # Move the robot to each waypoint
        for i in range(len(waypoints) - 1):
            start_time = time.time()

            # Calculate the distance and direction to the next waypoint
            dx = (waypoints[i + 1][1] - waypoints[i][1]) * meters_per_pixel
            dy = (waypoints[i + 1][0] - waypoints[i][0]) * meters_per_pixel
            distance = math.sqrt(dx ** 2 + dy ** 2)
            direction = math.atan2(dy, dx)

            # Convert the direction to degrees
            direction_degrees = math.degrees(direction)

            # Calculate the new position of the robot
            new_position = self.current_position + rotate_by_yaw(np.array([distance, 0.0, 0.0]), direction_degrees)

            # Update the robot's position and orientation
            self.current_position = new_position
            self.current_yaw = direction_degrees

            # Wait for the remaining time to achieve the desired time step
            elapsed_time = time.time() - start_time
            if elapsed_time < time_step:
                time.sleep(time_step - elapsed_time)

            self.window.update()

        self.current_yaw = 0
        self.reset_robot_pose(self.current_position, get_orientation_from_yaw(self.current_yaw))

    def initialize_tk_window(self):
        self.window = tk.Tk()
        self.window.title("Robot Control")

        image_frame = tk.Frame(self.window)
        image_frame.grid(row=0, column=0, padx=10, pady=10)

        self.following_label_rgb = tk.Label(image_frame)
        self.following_label_rgb.grid(row=0, column=0, padx=5, pady=5)

        following_rgb_text = tk.Label(image_frame, text="RGB Camera")
        following_rgb_text.grid(row=1, column=0, padx=5, pady=5)

        self.following_label_depth = tk.Label(image_frame)
        self.following_label_depth.grid(row=0, column=1, padx=5, pady=5)

        following_depth_text = tk.Label(image_frame, text="LIDAR")
        following_depth_text.grid(row=1, column=1, padx=5, pady=5)

        button_frame = tk.Frame(self.window)
        button_frame.grid(row=1, column=0, padx=10, pady=10)

        forward_button = tk.Button(button_frame, text="Forward", command=self.move_forward)
        forward_button.pack(side=tk.LEFT)

        backward_button = tk.Button(button_frame, text="Backward", command=self.move_backward)
        backward_button.pack(side=tk.LEFT)

        left_button = tk.Button(button_frame, text="Left", command=self.turn_left)
        left_button.pack(side=tk.LEFT)

        right_button = tk.Button(button_frame, text="Right", command=self.turn_right)
        right_button.pack(side=tk.LEFT)

        self.window.bind("<Up>", self.move_forward)
        self.window.bind("<Down>", self.move_backward)
        self.window.bind("<Left>", self.turn_left)
        self.window.bind("<Right>", self.turn_right)

