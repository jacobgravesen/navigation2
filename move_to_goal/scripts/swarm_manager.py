#! /usr/bin/env python3

import cv2
import rclpy
from position_infomation import PositionInfo as Info
from robot_navigator import NavigationResult



def main():
    rclpy.init()

    image = cv2.imread(r'/home/$USER/navigation2/move_to_goal/images/factory_map_CV.pgm')

    num_robots = 3

    robot_list = Info.gen_lists(num_robots)[0]
    initial_list = Info.gen_lists(num_robots)[1]
    goal_list = Info.gen_lists(num_robots)[2]
    eta_list = Info.gen_lists(num_robots)[3]

    # List of known points outside obstacles...
    goals = [[[1.5, 11.0], [16.5, 9.0]], [[5.0, 14.0], [9.0, 9.0]], [[11.0, 17.0], [1.5, 11.0]], [[9.0, 9.0], [5.0, 14.0]], [[16.5, 9.0], [11.0, 17.0]]]

    # Fix image so it does not create goals inside obstacles...
    # goals = Info.generate_random_goals(num_robots, image)

    Info.set_initial_pose(robot_list, initial_list, num_robots)

    Info.publish_goals(robot_list, goal_list, goals, num_robots)


    # sanity check a valid path exists: Make this work with multiple robots...
    # path = robot_list[i].getPath(initial_pose, goal_pose)


    # Make breakout function for when every robot has completed its goal...
    # Doesn't work at the moment........
    # i = 0
    # while True:
    #     Info.get_feedback(robot_list, eta_list, num_robots)



    # Do something depending on the return code: Make this work with multiple robots...
    # result = robot_list[i].getResult()
    # if result == NavigationResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == NavigationResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == NavigationResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')




if __name__ == '__main__':
    main()
