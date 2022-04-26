#! /usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_navigator import BasicNavigator, NavigationResult

import cv2
from action_msgs.msg import GoalStatus
import numpy as np
import random


class PositionInfo(Node):
    def __init__(self):
        super().__init__(node_name='position_info')
        # self.initial_pose = PoseStamped()
        # self.initial_pose.header.frame_id = 'map'

        # self.goal_handle = None
        # self.result_future = None
        # self.feedback = None
        # self.status = None
        # self.namespace = ''

    def generate_lists(self, num_robots): #0 = robot_list, 1 = initial_list, 2 = goal_list, 3 = next_goal_list
        robot_list = {}
        initial_list = {}
        goal_list = {}
        next_goal_list = {}

        for i in range(1, num_robots + 1):
            robot_list[f"robot{i}"] = BasicNavigator()
            initial_list[f"robot{i}"] = PoseStamped()
            goal_list[f"robot{i}"] = PoseStamped()
            next_goal_list[f"robot{i}"] = 0

        return robot_list, initial_list, goal_list, next_goal_list    

    def generate_initial_pose(self, robot_list, initial_list):
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

    def generate_random_goals(self, num_robots, image):

        # Size of image in pixels (x,y)
        image_size = [image.shape[0], image.shape[1]]

        goal = []
        conversion = image_size[0]/19.9 #19.9 is the equivalent distance in gazebo units.

        # 1-5 random goals are created for each robot
        for i in range(1, num_robots + 1): 
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
    
    def publish_goals_iteratively(self, robot_list, goal_list, goal_x, goal_y, goal_w):
        for i in range(1, len(robot_list) + 1):
            goal_list[f"robot{i}"].header.frame_id = 'map'
            goal_list[f"robot{i}"].header.stamp = robot_list[f"robot{i}"].get_clock().now().to_msg()
            goal_list[f"robot{i}"].pose.position.x = goal_x
            goal_list[f"robot{i}"].pose.position.y = goal_y
            goal_list[f"robot{i}"].pose.orientation.w = goal_w
            robot_list[f"robot{i}"].goToPose(goal_list[f"robot{i}"])

    
    def publish_goals(self, robot_list, goal_list, goal_x, goal_y, goal_w, robot):
            goal_list[f"robot{robot}"].header.frame_id = 'map'
            goal_list[f"robot{robot}"].header.stamp = robot_list[f"robot{robot}"].get_clock().now().to_msg()
            goal_list[f"robot{robot}"].pose.position.x = goal_x
            goal_list[f"robot{robot}"].pose.position.y = goal_y
            goal_list[f"robot{robot}"].pose.orientation.w = goal_w
            robot_list[f"robot{robot}"].goToPose(goal_list[f"robot{robot}"])

    def create_goals(self, robot_list, goal_list, goals, robot):
        goal_poses = []
        for k in range(len(goals[robot-1])):
            goal_list[f"robot{robot}"].header.frame_id = 'map'
            goal_list[f"robot{robot}"].header.stamp = robot_list[f"robot{robot}"].get_clock().now().to_msg()
            goal_list[f"robot{robot}"].pose.position.x = goals[robot-1][k][0]
            goal_list[f"robot{robot}"].pose.position.y = goals[robot-1][k][1]
            goal_list[f"robot{robot}"].pose.orientation.w = 1.0
            goal_poses.append(goal_list[f"robot{robot}"])
        return goal_poses
