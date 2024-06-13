import numpy as np

class Go2Interface:
    def __init__(self):
        self.rgb_image = np.zeros((256, 256, 3), dtype=np.uint8)
        # self.lidar_array = [1] * 256
        self.lidar_array = np.zeros((256, 256, 3), dtype=np.uint8)
        self.robot_vel_xy = np.zeros(2)
    
    def get_rgb_image(self):
        return self.rgb_image

    def get_lidar_array(self):
        return self.lidar_array
    
    def get_robot_vel_xy(self):
        return self.robot_vel_xy