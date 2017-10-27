#!/usr/bin/env python
#encoding = utf-8

import rospy
import actionlib
import roslib
import dji_sdk.msg
import dji_sdk.srv

class Drone(object):

	def flight_control_callback(self):
		pass

	def init_flight_control(self):
		self.flight_control_subscriber = rospy.Subscriber("dji_sdk/flight_control_setpoint_generic", )