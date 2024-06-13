from utils.teleop_visualizer import TeleopVisualizer
from utils.go2_test_interface import Go2Interface
import time


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