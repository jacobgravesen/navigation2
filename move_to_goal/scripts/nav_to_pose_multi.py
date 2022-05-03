#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
import rclpy
import time
from robot_navigator import BasicNavigator, NavigationResult

import cv2
import numpy as np
import math
import random

from action_msgs.msg import GoalStatus


image = cv2.imread(r'/home/william/Pictures/factory_map_CV.pgm') #Billedet blir loadet med opencv

def generate_random_goals(num_robots):

    #Size of image in pixels (x,y)
    image_size = [395, 406]

    goal = []
    conversion = image_size[0]/19.9 #19.9 is the equivalent distance in gazebo units.

    for i in range(1, num_robots + 1): #Her blir der lavet random slut punkter til robotterne (der blir lavet random 1-5 random punkter for hver robot der er)
        rand = int(random.uniform(1, 5))
        localgoal = []
        for j in range(rand):
            while True:
                x = int(random.uniform(1, image_size[0]))
                y = int(random.uniform(1, image_size[1]))
                if np.any(image[x, y] != 0):
                    conx = round(x / conversion, 2)
                    cony = round(y / conversion, 2)
                    localgoal.append([conx, cony])
                    break
        goal.append(localgoal)

    return goal

def generate_robot_list(num_robots):

    robot_list = []

    for i in range(1, num_robots + 1):
        robot_name = "robot" + str(i)
        robot_name = BasicNavigator()
        x_pos = float(i)
        robot_list.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.5, 'z_pose': 0.01})

    return robot_list



def main():
    rclpy.init()

    # Define number of robots
    robot_num = 3

    goal = generate_random_goals(robot_num)
    # Populate lists with namespaced class instances
    robot_list = {}
    initial_list = {}
    goal_list = {}
    for i in range(1, robot_num + 1):
        robot_list[f"robot{i}"] = BasicNavigator()
        initial_list[f"robot{i}"] = PoseStamped()
        goal_list[f"robot{i}"] = PoseStamped()

    for i in range(1, len(robot_list) + 1):
        namespace = "robot" + str(i) + "/"
        robot_list[f"robot{i}"].createSubscriptions(namespace)
        initial_list[f"robot{i}"].header.frame_id = 'map'
        initial_list[f"robot{i}"].header.stamp = robot_list[f"robot{i}"].get_clock().now().to_msg()
        initial_list[f"robot{i}"].pose.position.x = float(i) - 1
        initial_list[f"robot{i}"].pose.position.y = -0.5
        initial_list[f"robot{i}"].pose.orientation.z = 0.0 # Just initial, may not need to be changed
        initial_list[f"robot{i}"].pose.orientation.w = 1.0 # Just initial, may not need to be changed
        robot_list[f"robot{i}"].setInitialPose(initial_list[f"robot{i}"])

    for i in range(1, len(robot_list) + 1):
        robot_list[f"robot{i}"].waitUntilNav2Active(namespace)  

    for i in range(1, len(robot_list) + 1):
        goal_list[f"robot{i}"].header.frame_id = 'map'
        goal_list[f"robot{i}"].header.stamp = robot_list[f"robot{i}"].get_clock().now().to_msg()
        goal_list[f"robot{i}"].pose.position.x = goal[i-1][0][0]
        goal_list[f"robot{i}"].pose.position.y = goal[i-1][0][1]
        goal_list[f"robot{i}"].pose.orientation.w = 1.0 # Might be useful to change this depending on carrier agent
        robot_list[f"robot{i}"].goToPose(goal_list[f"robot{i}"])
        time.sleep(5.0)


    i = 0
    #while not robot_list[f"robot{1}"].isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
    while True:
        i = i + 1
        feedback = robot_list[f"robot{1}"].getFeedback()
        print(feedback)
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

    # Do something depending on the return code
    result = robot_list[f"robot{1}"].getResult()
    if result == GoalStatus.STATUS_SUCCEEDED:
        print('Goal succeeded!')
        print("result is: "+str(result))
    elif result == GoalStatus.STATUS_CANCELED:
        print('Goal was canceled!')
    elif result == GoalStatus.STATUS_ABORTED:
        print('Goal failed!')
    #else:
    #    print('Unexpected error')





if __name__ == '__main__':
    main()
