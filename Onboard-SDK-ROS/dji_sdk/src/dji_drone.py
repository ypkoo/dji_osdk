#!/usr/bin/env python
#encoding = utf-8

import rospy
import roslib

from sensor_msgs.msg import Joy, BatteryState, NavSatFix
from geomerty_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped
from std_msgs.msg import UInt8
from dji_sdk.msg import *
from dji_sdk.srv import *

class Drone(object):

	def __init__(self):
		rospy.init_node('dji_sdk_ros_python')

		self.attitude = QuaternionStamped()
		self.flight_status = Uint8()
		self.battery_state = BatteryState()
		self.velocity = Vector3Stamped()
		self.gps_health = Uint8()
		self.gps_position = NavSatFix()
		self.local_position = PointStamped()

		self.init_services()
		self.init_subscribers()
		self.init_publishers()

	def init_services(self):
		rospy.wait_for_service("dji_sdk/activation")
		rospy.wait_for_service("dji_sdk/drone_arm_control")
		rospy.wait_for_service("dji_sdk/drone_task_control")
		rospy.wait_for_service("dji_sdk/query_drone_version")
		rospy.wait_for_service("dji_sdk/sdk_control_authority")
		rospy.wait_for_service("dji_sdk/set_local_pos_ref")

		self.activationService = rospy.ServiceProxy("dji_sdk/activation", Activation)
		self.droneArmControlService = rospy.ServiceProxy("dji_sdk/drone_arm_control", DroneArmControl)
		self.droneTaskControlService = rospy.ServiceProxy("dji_sdk/drone_task_control", DroneTaskControl)
		self.queryDroneVersionService = rospy.ServiceProxy("dji_sdk/query_drone_version", QueryDroneVersion)
		self.sdkControlAuthorityService = rospy.ServiceProxy("dji_sdk/sdk_control_authority", SDKControlAuthority)
		self.setLocalPosRefService = rospy.ServiceProxy("dji_sdk/set_local_pos_ref", SetLocalPosRef)

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

	def activate(self):
		result = self.activationService()
		return result.result

	def drone_version(self):
		result = self.queryDroneVersionService()
		return result.result

	def arm(self):
		result = self.droneArmControlService(arm=1)
		return result.result

	def disarm(self):
		result = self.droneArmControlService(arm=0)
		return result.result

	def takeoff(self):
		result = self.droneTaskControlService(task=TASK_TAKEOFF)
		return result.result

	def land(self):
		result = self.droneTaskControlService(task=TASK_LAND)
		return result.result

	def gohome(self):
		result = self.droneTaskControlService(task=TASK_GOHOME)
		return result.result

	def request_sdk_control(self):
		result = self.sdkControlAuthorityService(control_enbale=REQUEST_CONTROL)
		return result.result

	def release_sdk_control(self):
		result = self.sdkControlAuthorityService(control_enable=RELEASE_CONTROL)
		return result.result

	def set_local_position_reference(self):
		result = self.setLocalPosRefService()
		return result.result

	def flight_status_callback(self, flight_status):
		self.flight_status = flight_status

	def attitude_callback(self, attitude):
		self.attitude = attitue

	def battery_state_callback(self, battery_state):
		self.battery_state = battery_state

	def velocity_callback(self, velocity):
		self.velocity = velocity

	def gps_health_callback(self, gps_health):
		self.gps_health = gps_health

	def gps_position_callback(self, gps_position):
		self.gps_position = gps_position

	def local_position_callback(self, local_position):
		self.local_position = local_position
