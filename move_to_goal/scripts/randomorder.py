import cv2
import numpy as np
import math
import random

image = cv2.imread(r'/home/william/Pictures/factory_map_CV.pgm') #Billedet blir loadet med opencv


#Size of image in pixels (x,y)
image_size = [395, 406]

goal = []
conversion = image_size[0]/19.9 #19.9 is the equivalent distance in gazebo units.

for i in range(1, 2 + 1): #Her blir der lavet random slut punkter til robotterne (der blir lavet random 1-5 random punkter for hver robot der er)
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

print(len(goal[0]))
print(goal)
print(goal[0])
print(goal[0][0])
print(goal[0][0][0])