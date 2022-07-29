#!/usr/bin/env python3

'''
Displays the map of waypoints
'''

# Imports
import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg
from rospkg import RosPack
from nav_msgs.msg import Odometry
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import matplotlib.pyplot as plt
from matplotlib import patches

# GLOBAL VARIABLES 
xc = 0
yc = 0
yaw = 0 
idx = 0
waypoints = []

def read_points():
   # CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
   r = rospkg.RosPack()
   package_path = r.get_path('pure_pursuit')
   file_name = 'barca.csv' #'racecar_walker.csv'
   file_path = package_path + '/waypoints/' + file_name
   with open(file_path) as f: # see if i can open it  as a numpy array
      path_points = np.loadtxt(file_path, delimiter = ',')
   return path_points

if __name__=='__main__':
   waypoints = read_points()
   plt.cla()
   #PURE PURSUIT CODE 
   cx = []; cy = []
   for point in waypoints:
      cx.append(float(point[0]))
      cy.append(float(point[1]))
   plt.plot(cx, cy, "-r", label = "course")
   plt.axis("equal")
   plt.grid(True)
   plt.title("Pure Pursuit Control" + str(1))
   plt.show()
