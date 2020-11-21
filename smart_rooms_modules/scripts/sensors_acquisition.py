#!/usr/bin/python
import RPi.GPIO as GPIO
import time, sys, signal
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

class Sensors():
	def __init__(self):
		rospy.init_node('sensor_acquisition', anonymous = True)
		rospy.loginfo("Starting Sensors Acquisition")
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		#self.initServiceClients()
		self.initVariables()
		self.mainControl()
    
	def initParameters(self):
		self.room_name = rospy.get_param("~room_name","room1")
		self.read_magnetic = rospy.get_param("~magnetic_sensor/read",True)
		self.read_door_lasers = rospy.get_param("~door_laser_sensors/read",False)
		self.door_lasers_addresses = rospy.get_param("~door_laser_sensors/addresses",[0x29])
		self.door_lasers_names = rospy.get_param("~door_laser_sensors/names",["left"])
		self.node_rate = rospy.get_param("~node_paramters/rate",100)
		return

	def initSubscribers(self):
		self.subKillNode = rospy.Subscriber("/kill_sensors_acquisition", Bool, self.callbackKillNode)

	def initPublishers(self):
		if self.read_magnetic:
			rospy.loginfo("Acquiring Magnetic Sensor")
			topic_name = "/" + self.room_name + "/magnetic_sensor_state"
			self.pubMgn = rospy.Publisher(topic_name, PointStamped, queue_size = 10)
		if self.read_door_lasers:
			i2c_bus = subprocess.Popen(['i2cdetect', '-y', '1',], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			stdout, stderr = i2c_bus.communicate()
			devices_connected = stdout
			for i in range(len(self.door_lasers_addresses)):
				if (self.door_lasers_addressess[i] in devices_connected):
					rospy.loginfo("Acquiring Door Laser Sensor '%s'",self.door_lasers_names[i])
					topic_name = "/" + self.room_name + "/" + self.door_lasers_names[i] + "_laser_sensor_state"
					self.pubDoorLaser[i] = rospy.Publisher(topic_name, Bool, queue_size = 10)

	def initVariables(self):
		self.rate = rospy.Rate(self.node_rate)
		self.magnetic_sensor_pin = 10
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.magnetic_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		self.kill_node = False

	def callbackKillNode(self,flag):
		self.kill_node = flag.data

	def mainControl(self):
		rospy.loginfo("Sensors Acquisition in %s: OK", self.room_name)
		while not rospy.is_shutdown():
			msg = PointStamped()
			msg.header.frame_id = self.room_name
			msg.point.x =  GPIO.input(self.magnetic_sensor_pin)
			self.pubMgn.publish(msg)
			if self.kill_node:
				break
			self.rate.sleep()
		rospy.loginfo("Sensors Acquisition in %s: Finished", self.room_name)

if __name__ == '__main__':
	try:
		s = Sensors()
	except rospy.ROSInterruptException:
		rospy.loginfo("Sensors Acquisition in %s: Finished", self.room_name)		
		pass
