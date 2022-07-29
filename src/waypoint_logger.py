#!/usr/bin/env python3

'''
Use this to create a lists of waypoints
'''

import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import time
import matplotlib.animation as animation
from matplotlib import style
import rospkg 

home = expanduser('~')
# CHANGE THIS PATH TO WHERE YOU WANT TO SAVE YOUR CSV FILE 
r = rospkg.RosPack()
file_path=r.get_path('pure_pursuit')
file_name='wp_file.csv'
file = open(file_path+'/waypoints/'+file_name,'w')
ctr = 0 
print("saving to file pure_pursuit/waypoints with a name - ", file_name)

def save_waypoint(data):
	global ctr
	quaternion = np.array([data.pose.pose.orientation.x, 
				data.pose.pose.orientation.y, 
				data.pose.pose.orientation.z, 
				data.pose.pose.orientation.w])
	euler = tf.transformations.euler_from_quaternion(quaternion)
	speed = LA.norm(np.array([data.twist.twist.linear.x, 
				   data.twist.twist.linear.y, 
				   data.twist.twist.linear.z]),2)
	
	if data.twist.twist.linear.x > 0.:
		pass
		#print data.twist.twist.linear.x
	if ctr == 10:
		file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x, data.pose.pose.position.y, euler[2], speed))
		print(data.pose.pose.position.x, data.pose.pose.position.y)
		ctr = 0 
	else: 
		ctr +=1 
	

def shutdown():
	file.close()
	print('Goodbye')

def listener():
	rospy.init_node('waypoints_logger', anonymous=True)
	rospy.Subscriber('/vesc/odom', Odometry, save_waypoint)
	rospy.spin()

if __name__ == '__main__':
	atexit.register(shutdown)
	print('Saving waypoints...')
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
