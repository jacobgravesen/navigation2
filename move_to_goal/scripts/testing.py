#! /usr/bin/env python3
from time import sleep
from position_infomation import PositionInfo
import rclpy
from robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import cv2
from action_msgs.msg import GoalStatus
import threading
from rclpy.duration import Duration


def main():
    # Full path to image
    image = cv2.imread(r'/home/william/Pictures/factory_map_CV.pgm')

    rclpy.init()

    # Number of robots
    num_robots = 3

    # Instanciate the class
    instance = PositionInfo()

    # Create essential lists
    robot_list = instance.generate_lists(num_robots)[0]
    initial_list = instance.generate_lists(num_robots)[1]
    goal_list = instance.generate_lists(num_robots)[2]
    eta_list = instance.generate_lists(num_robots)[3]

    # Create initial poses for each robot
    instance.generate_initial_pose(robot_list, initial_list)

    # Create random goals for each robot
    #goals = instance.generate_random_goals(num_robots, image)

    goal_w = 1.0 #Incorporate somehow!

    goals = [[[1.5, 11.0], [16.5, 9.0]], [[5.0, 14.0], [9.0, 9.0]], [[11.0, 17.0], [1.5, 11.0]], [[9.0, 9.0], [5.0, 14.0]], [[16.5, 9.0], [11.0, 17.0]]]

    goal_pose = {}
    for i in range(1, len(robot_list) + 1):
        goal_pose[f"robot{i}"] = instance.create_goals(robot_list, goal_list, goals, i)
        robot_list[f"robot{i}"].goToPose(goal_pose[f"robot{i}"][1])
        sleep(2)



    j = 0
    while True:
        print("In while true")
        for i in range(1, len(robot_list) + 1):
            
            j += 1
            feedback = robot_list[f"robot{i}"].getFeedback()
            print(feedback)

            if feedback and j % 5 == 0:
                eta_list[f"time_robot{i}"] = '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)


main()



