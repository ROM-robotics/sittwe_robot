#!/usr/bin/env python
import rospy
import sys
import time
import math

from SerialDataGateway import SerialDataGateway
from std_msgs.msg import String
from sittwe_driver.msg import rpm
from sittwe_driver.msg import actual_rpm

class Arduino_Class(object):

	def __init__(self):
		print "Initializing Arduino Class"

		self.desire_right_rpm = 0
		self.desire_left_rpm = 0

		self.actual_right_rpm = 0
		self.actual_left_rpm = 0
		self.delta_time = 0; 
		
		self.actual_rpms = actual_rpm()
		self._Counter = 0

		port = rospy.get_param("~port", "/home/ubuntu/romrobotics")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")

		self._Desire_Subscriber = rospy.Subscriber('desire_rpm',rpm,self._Handle_rpm_request)
		self._Actual_Publisher = rospy.Publisher('actual_rpm',actual_rpm,queue_size = 50)	

		# for debugging
		#self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)

	def _Handle_rpm_request(self, msg):
		self.desire_right_rpm = msg.desire_rpm_right
		self.desire_left_rpm  = msg.desire_rpm_left
		speed_message = '%d,%d,%d \r' %(int(1),int(self.desire_right_rpm),int(self.desire_left_rpm))		
		self._WriteSerial(speed_message)
		#rospy.loginfo("_Handle_rpm_request")
		#rospy.loginfo(speed_message)


	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		#self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))
		if(len(line) > 0):
			lineParts = line.split(',')
			li = lineParts
			ros_time = rospy.Time.now()
			try:
				if(li[0] == '5'):
					self.actual_right_rpm = float(li[1])
					self.actual_left_rpm = float(li[2])
					self.delta_time  = float(li[3])
					imu_yaw = float(li[4])
     
					self.actual_rpms.actual_right = self.actual_right_rpm
					self.actual_rpms.actual_left = self.actual_left_rpm
					self.actual_rpms.delta_time = self.delta_time;
					self.actual_rpms.imu_yaw = imu_yaw;
					self._Actual_Publisher.publish(self.actual_rpms)

			except:
				rospy.loginfo("exception!")
				pass

	def _WriteSerial(self, message):
		#self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)
		#rospy.loginfo("_WriteSerial")
		#rospy.loginfo(message)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()



if __name__ =='__main__':
	rospy.init_node('arduino_ros',anonymous=False)
	Arduino = Arduino_Class()
	try:

		Arduino.Start()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")
		
	Arduino.Stop()

#######################################################################################################################
