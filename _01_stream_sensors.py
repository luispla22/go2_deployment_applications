from utils.teleop_visualizer import TeleopVisualizer
from utils.go2_test_interface import Go2Interface
import time


def main():

    teleop_visualizer = TeleopVisualizer()
    go2_interface = Go2Interface()

    for i in range(10000):
        rgb_image = go2_interface.get_rgb_image()
        lidar_data = go2_interface.get_lidar_array()

        teleop_visualizer.set_rgb_image(rgb_image)
        teleop_visualizer.set_lidar_data(lidar_data)

        teleop_visualizer.step()

        time.sleep(0.01)


if __name__ == "__main__":
    main()