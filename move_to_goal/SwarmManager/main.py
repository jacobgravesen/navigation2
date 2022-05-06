import random
import time

from kivy.app import App
from kivy.metrics import dp
from kivy.core.window import Window
from kivy.properties import StringProperty, ObjectProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.button import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.screenmanager import Screen, ScreenManager
from time import sleep
import threading
from swarm_manager import initial_poses, go_to_goals, generate_random_goalsz
from kivy.clock import Clock

import position_infomation # Importing Positions for ROS
from position_infomation import PositionInfo
import cv2

# ROS Imports
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

from robot_navigator import BasicNavigator

class MainMenu(BoxLayout, Screen):
    pass

class TurtleBotLogo(BoxLayout):
    pass

class MainMenuOptionSwarmInfo(BoxLayout, Screen): # This is the Menu Option that displays Swarm Info.
    def swarm_manager(self):
        print("Swarm Is Commencing")
        print(str(eta_list[f"time_robot{1}"]))
    pass

class MainMenuOptionSwarmInfoBottom(BoxLayout):
    def generate_random_goals(self):
        goals = generate_random_goalsz(num_robots)
        print(goals)

        # print(generate_random_goalsz(num_robots))

    def go_to_goals_threaded(self):
        go_to_goals_thread = threading.Thread(target=go_to_goalsz)
        go_to_goals_thread.start()

    #def get_feedback(self):
    #    get_feedback(eta_list, robot_list)
    #    print("getting feedback")
    def threaded_get_feedback(self):
        thread1 = threading.Thread(target=get_feedback)
        thread1.start()

    def check_if_ready(self, dt):
        print(robot_list[f"robot{1}"])


    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_interval(self.check_if_ready, 1)


class SwarmMenuRobotInfo(GridLayout):
    #name = StringProperty()
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_interval(self.press, 1)
    def press(self, dt):
        robots_array = []
        for robot in range(1, num_robots + 1):


            # Create variables for our widget
            name = eta_list[f"time_robot{robot}"]
            robots_array.append(name)

        if robots_array[0]:
            self.ids.Robot1.text = "ETA of Robot1 is: " + str(robots_array[0])
        if robots_array[1]:
            self.ids.Robot2.text = "ETA of Robot2 is: " + str(robots_array[1])
        try:
            if robots_array[2]:
                self.ids.Robot1.text = "ETA of Robot3 is: " + str(robots_array[2])
            if robots_array[3]:
                self.ids.Robot2.text = "ETA of Robot4 is: " + str(robots_array[3])
            if robots_array[4]:
                self.ids.Robot1.text = "ETA of Robot5 is: " + str(robots_array[4])
            if robots_array[5]:
                self.ids.Robot2.text = "ETA of Robot6 is: " + str(robots_array[5])
            if robots_array[6]:
                self.ids.Robot1.text = "ETA of Robot7 is: " + str(robots_array[6])
            if robots_array[7]:
                self.ids.Robot2.text = "ETA of Robot8 is: " + str(robots_array[7])
        except:
            print("tried")
        print(robots_array)

class MainMenuOptionInitializeSwarm(BoxLayout, Screen):
    def set_initial_poses_threaded(self):
        set_initial_poses_thread = threading.Thread(target=set_initial_poses)
        set_initial_poses_thread.start()

class HLButton(Button): # This Button Class Lights Up When Hovered Over.

    def __init__(self, *args, **kwargs):
        super(HLButton, self).__init__(*args, **kwargs)
        Window.bind(mouse_pos=self.pos_check)

    def pos_check(self, inst, pos):
        if self.collide_point(*pos):
             self.background_color = (0.13, 0.126, 0.51, 1)
        else:
             self.background_color = (0.129, 0.129, 0.133, 1)

class SwarmManagerApp(App): # This is the main "SwarmManagerApp" class that runs everything.
    pass

###########################
##### ROS DEFINITIONS #####
###########################

# Generate lists to be used by the swarm Manager.

def get_feedback():
    i = 0
    while True:
        for robot in range(1, len(robot_list) + 1):

            while not robot_list[f"robot{robot}"].isNavComplete():

                # Do something with the feedback
                i = i + 1
                feedback = robot_list[f"robot{robot}"].getFeedback()
                if feedback and i % 5 == 0:

                    eta_list[f"time_robot{robot}"] = '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    print("ETA of robot" + str(robot) + " " '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        print("WOW, it takes more than 10 minutes to go to this goal!")
                break

def swarm_initializer(num_robots):
    global robot_list
    global initial_list
    global goal_list
    global eta_list


    robot_list = {}
    initial_list = {}
    goal_list = {}
    eta_list = {}

    for i in range(1, num_robots + 1):
        robot_list[f"robot{i}"] = BasicNavigator()
        initial_list[f"robot{i}"] = PoseStamped()
        goal_list[f"robot{i}"] = PoseStamped()
        eta_list[f"time_robot{i}"] = str()

    return robot_list, initial_list, goal_list

def menu_start_function(): # J.G. is the ultimate hacker.
    SwarmManagerApp().run()

def rclpy_thread():
    rclpy.init()

def screen_manager():
    sm = ScreenManager()
    sm.add_widget(MainMenu(name='mainmenu'))

def go_to_goalsz():
    go_to_goals(robot_list, goal_list)
    print("going to goals")

def set_initial_poses():
    initial_poses(robot_list, initial_list)

threading.Thread(target=rclpy_thread()).start()
global num_robots
num_robots = 7
swarm_initializer(num_robots)
# swarm_initializer(num_robots)
# generate_lists(num_robots) # Generate lists to be used by the swarm Manager.

screen_manager_thread = threading.Thread(target=screen_manager())
screen_manager_thread.start()

main_kivy = threading.Thread(target=menu_start_function())
main_kivy.start()  # This is the instanceiation of the main "SwarmManagerApp" class.

