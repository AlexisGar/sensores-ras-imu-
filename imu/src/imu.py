#!/usr/bin/env python

import os, sys
import subprocess
import rospy
from sensor_msgs.msg import Imu

class imu ():
	def __init__(self):
		
		#IMU Message variables
		self.seq = 0
		self.frame_id = rospy.get_param('~frame_id', 'imu_link')
		
		#Publisher
		self.pub = rospy.Publisher('dof', Imu, queue_size=10)
		
		rate = rospy.Rate(2)
		
		while not rospy.is_shutdown():
			self.reading()			
			rate.sleep()

	def reading(self):
		
		
		data = subprocess.Popen("minimu9-ahrs --mode raw", shell=True, stdout=subprocess.PIPE)		
		while data.poll() is None:
			data_final = data.stdout.readline()
			x = str(data_final.decode(sys.getdefaultencoding()).rstrip())#.split(' ') #revisar velocidad de lectura, en mov no lee ok
			x= x.split(' ')
			#Remove white spaces from the list		
			z=filter(lambda a: a != '',x)
			x=z
			#Assign the read values to ROS message		
			mx = float(x[0])
			my = float(x[1])
			mz = float(x[2])
			gx = float(x[3])
			gy = float(x[4])
			gz = float(x[5])
			ax = float(x[6])
			ay = float(x[7])
			az = float(x[8])
			i = Imu()	
			i.header.stamp = rospy.Time.now()
			i.header.frame_id = self.frame_id
			i.header.seq = self.seq

			#Magnetometer
			i.orientation.x = mx 
			i.orientation.y = my
			i.orientation.z = mz

			#Gyroscope
			i.angular_velocity.x = gx
			i.angular_velocity.y = gy
			i.angular_velocity.z = gz
		
			#Accelerometer		
			i.linear_acceleration.x = ax
			i.linear_acceleration.y = ay
			i.linear_acceleration.z = az
			rospy.loginfo(i)
			self.seq += 1
			self.pub.publish(i)


if __name__ == '__main__':
	rospy.init_node('imu', anonymous=True)
	try:	
		imu()
	except rospy.ROSInterruptException:
		pass
