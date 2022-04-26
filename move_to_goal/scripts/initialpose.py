#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
import rclpy
from robot_navigator import BasicNavigator


def main():
    rclpy.init()

    # Define number of robots
    robot_num = 2

    # Populate lists with namespaced class instances
    robot_list = {}
    initial_list = {}
    for i in range(1, robot_num + 1):
        robot_list[f"robot{i}"] = BasicNavigator()
        initial_list[f"robot{i}"] = PoseStamped()

    for i in range(1, len(robot_list) + 1):
        namespace = "robot" + str(i) + "/"
        robot_list[f"robot{i}"].createSubscriptions(namespace)
        initial_list[f"robot{i}"].header.frame_id = 'map'
        initial_list[f"robot{i}"].header.stamp = robot_list[f"robot{i}"].get_clock().now().to_msg()
        initial_list[f"robot{i}"].pose.position.x = float(i)
        initial_list[f"robot{i}"].pose.position.y = 0.5
        initial_list[f"robot{i}"].pose.orientation.z = 0.0 # Just initial, may not need to be changed
        initial_list[f"robot{i}"].pose.orientation.w = 1.0 # Just initial, may not need to be changed
        robot_list[f"robot{i}"].setInitialPose(initial_list[f"robot{i}"])

    for i in range(1, len(robot_list) + 1):
        robot_list[f"robot{i}"].waitUntilNav2Active(namespace)



if __name__ == '__main__':
    main()
