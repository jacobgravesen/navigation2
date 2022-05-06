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

def swarm_initializer(num_robots):
    global robot_list
    global initial_list
    global goal_list

    robot_list = {}
    initial_list = {}
    goal_list = {}

    for i in range(1, num_robots + 1):
        robot_list[f"robot{i}"] = BasicNavigator()
        initial_list[f"robot{i}"] = PoseStamped()
        goal_list[f"robot{i}"] = PoseStamped()

    return robot_list, initial_list, goal_list

def initial_poses(robot_list, initial_list):
    # Create initial poses for each robot
    PositionInfo().generate_initial_pose(robot_list, initial_list)

# Create random goals for each robot
#goals = instance.generate_random_goals(num_robots, image)

def go_to_goals(robot_list, goal_list): #
    goal_w = 1.0 #Incorporate somehow!
    # goals = [[[1.5, 11.0], [16.5, 9.0]], [[5.0, 14.0], [9.0, 9.0]], [[11.0, 17.0], [1.5, 11.0]], [[9.0, 9.0], [5.0, 14.0]], [[16.5, 9.0], [11.0, 17.0]]]
    goal_pose = {}
    print("test1")
    for i in range(1, len(robot_list) + 1):
        goal_pose[f"robot{i}"] = PositionInfo().create_goals(robot_list, goal_list, goals, i)
        robot_list[f"robot{i}"].goThroughPoses(goal_pose[f"robot{i}"])
        sleep(0.5)
    print("test2")

def generate_random_goalsz(num_robots):
    image = cv2.imread(r'images/factory_map_CV.pgm')
    global goals
    goals = PositionInfo().generate_random_goals(num_robots, image)
    # print(goals)
    return goals




# instance.publish_goals(robot_list, goal_list, goals[0][0][0], goals[0][0][1], goal_w, 1)

# while not robot_list[f"robot{1}"].isNavComplete():
#     print("Not complete yet...")
#     sleep(5.0)
#     print(robot_list[f"robot{1}"].getResult())

# total_goals = 0
# total_left = 0
# for i in range(len(goals)):
#     total_goals += len(goals[i])
#     total_left += len(goals[i])

# activity_list = {}
# for i in range(1, len(robot_list) + 1):
#     activity_list[f"robot{i}"] = "idle"


# while total_left > 0:
#     for i in range(total_goals):
#         for j in range(1, len(robot_list) + 1):
#             if activity_list[f"robot{j}"] == "idle":
#                 task_list = goals[j-1]
#                 next_goal = task_list[0]
#                 instance.publish_goals(robot_list, goal_list, next_goal[0], next_goal[1], goal_w, j)
#                 print("Total goals for robot" + str(j) + ": " + str(len(goals[j-1])))
#                 activity_list[f"robot{j}"] = "moving"

#             #print("Robot " + str(j) + ": RESULT: " + str(robot_list[f"robot{j}"].getResult()))
#             if robot_list[f"robot{j}"].isNavComplete:
#                 print("Nav for robot" + str(j) + " is complete!")

#             if robot_list[f"robot{j}"].getResult() == GoalStatus.STATUS_SUCCEEDED:
#                 if task_list != False:
#                     task_list.pop(0)
#                     activity_list[f"robot{j}"] = "idle"
#                     total_left -= 1

#                 elif task_list == False:
#                     print("All goals for robot" + str(j) + " completed!")
#                     robot_list[f"robot{j}"].cancelNav()
#                     robot_list[f"robot{j}"].lifecycleShutdown()