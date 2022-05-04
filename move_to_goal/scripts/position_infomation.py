#! /usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_navigator import BasicNavigator, NavigationResult
from rclpy.duration import Duration

import cv2
from action_msgs.msg import GoalStatus
import numpy as np
import random


class PositionInfo(Node):
    def __init__(self):
        super().__init__(node_name='position_info')



    def gen_lists(num_robots):

        robot_list = {}
        initial_list = {}
        goal_list = {}
        eta_list = {}

        for robot in range(1, num_robots + 1):
            robot_list[f"robot{robot}"] = BasicNavigator("robot" + str(robot) + "/")
            initial_list[f"robot{robot}"] = PoseStamped()
            goal_list[f"robot{robot}"] = PoseStamped()
            eta_list[f"robot{robot}"] = str()
        
        return robot_list, initial_list, goal_list, eta_list



    def set_initial_pose(robot_list, initial_list, num_robots):
        for robot in range(1, num_robots + 1):
            initial_list[f"robot{robot}"].header.frame_id = 'map'
            initial_list[f"robot{robot}"].header.stamp = robot_list[f"robot{robot}"].get_clock().now().to_msg()
            initial_list[f"robot{robot}"].pose.position.x = float(robot) - 1
            initial_list[f"robot{robot}"].pose.position.y = -0.5
            initial_list[f"robot{robot}"].pose.orientation.z = 0.0
            initial_list[f"robot{robot}"].pose.orientation.w = 1.0
            robot_list[f"robot{robot}"].setInitialPose(initial_list[f"robot{robot}"])

        for robot in range(1, num_robots + 1):
            robot_list[f"robot{robot}"].waitUntilNav2Active("robot" + str(robot) + "/")



    def publish_goals(robot_list, goal_list, goals, num_robots):
        for robot in range(1, num_robots + 1):
            goal_poses = []
            for k in range(len(goals[robot-1])):
                goal_list[f"robot{robot}"].header.frame_id = 'map'
                goal_list[f"robot{robot}"].header.stamp = robot_list[f"robot{robot}"].get_clock().now().to_msg()
                goal_list[f"robot{robot}"].pose.position.x = goals[robot-1][k][0]
                goal_list[f"robot{robot}"].pose.position.y = goals[robot-1][k][1]
                goal_list[f"robot{robot}"].pose.orientation.w = 1.0
                goal_poses.append(goal_list[f"robot{robot}"])
            robot_list[f"robot{robot}"].goThroughPoses(goal_poses)



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

    
    def get_feedback(robot_list, eta_list, num_robots):
        for robot in range(1, num_robots + 1):
            
            while not robot_list[f"robot{robot}"].isNavComplete():

                # Do something with the feedback
                i = i + 1
                feedback = robot_list[f"robot{robot}"].getFeedback()
                if feedback and i % 5 == 0:
                    eta_list[f"robot_time{robot}"] = '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    print("ETA of robot" + str(robot) + " " '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        print("WOW, it takes more than 10 minutes to go to this goal!")
                break


 #   def check_path_validity():
 #       for robot in range(1, num_robots + 1):
