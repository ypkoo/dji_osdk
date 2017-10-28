#!/usr/bin/env python
#encoding = utf-8

import rospy
import actionlib
import roslib
import dji_sdk.msg
import dji_sdk.srv

from sensor_msgs.msg import Joy, BatteryState, NavSatFix
from geomerty_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped
from std_msgs.msg import UInt8
from dji_sdk.msg import *
import nav_msgs.msg
from dji_sdk.srv import *

class Drone(object):

	def init_services(self):
		rospy.wait_for_service("dji_sdk/activation")
		rospy.wait_for_service("dji_sdk/drone_arm_control")
		rospy.wait_for_service("dji_sdk/drone_task_control")
		rospy.wait_for_service("dji_sdk/query_drone_version")
		rospy.wait_for_service("dji_sdk/sdk_control_authority")
		rospy.wait_for_service("dji_sdk/set_local_pos_ref")

		self.activationService = rospy.ServiceProxy("dji_sdk/activation", Activation)
		self.droneArmControl = rospy.ServiceProxy("dji_sdk/drone_arm_control", DroneArmControl)
		self.droneTaskControl = rospy.ServiceProxy("dji_sdk/drone_task_control", DroneTaskControl)
		self.queryDroneVersion = rospy.ServiceProxy("dji_sdk/query_drone_version", QueryDroneVersion)
		self.sdkControlAuthority = rospy.ServiceProxy("dji_sdk/sdk_control_authority", SDKControlAuthority)
		self.setLocalPosRef = rospy.ServiceProxy("dji_sdk/set_local_pos_ref", SetLocalPosRef)

	def init_subscribers(self):
		rospy.Subscriber("dji_sdk/flight_status", UInt8, self.flight_status_callback)
		rospy.Subscriber("dji_sdk/attitude", QuaternionStamped, self.attitude_callback)
		rospy.Subscriber("dji_sdk/battery_state", BatteryState, self.battery_state_callback)
		rospy.Subscriber("dji_sdk/velocity", Vector3Stamped, self.velocity_callback)
		rospy.Subscriber("dji_sdk/gps_health", UInt8, self.gps_health_callback)
		rospy.Subscriber("dji_sdk/gps_position", NavSatFix, self.gps_position_callback)
		rospy.Subscriber("dji_sdk/local_position", PointStamped, self.local_position_callback)

	def init_publishers(self):
		self.flightCtrlGenericPublisher = rospy.Publisher("dji_sdk/flight_control_setpoint_generic", Joy)
		self.flightCtrlPosPublisher = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUposition_yaw", Joy)
		self.flightCtrlVelPublisher = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", Joy)

	def flight_status_callback(self):
		pass

	def attitude_callback(self):
		pass

	def battery_state_callback(self):
		pass

	def velocity_callback(self):
		pass

	def gps_health_callback(self):
		pass

	def gps_position_callback(self):
		pass

	def local_position_callback(self):
		pass
