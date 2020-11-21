#!/usr/bin/python
from datetime import datetime
import rospy
from std_msgs.msg import Bool, String, Int16
from geometry_msgs.msg import PointStamped
from smart_rooms_msgs.msg import RoomStamped, RoomStampedList

class Orchestrator():
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
		self.rooms_list = rospy.get_param("rooms_list")
		return

	def initSubscribers(self):
		self.subKillNode = rospy.Subscriber("/kill_room_watcher", Bool, self.callbackKillNode)
		self.active_rooms = []
		for i in range(len(self.rooms_list)):
			key_name = "room" + str(i+1)
			if self.rooms_list[key_name]["active"]:
				self.active_rooms.append(key_name)
				topic_name = "/" + self.rooms_list[key_name]["name"] + "/status"
				self.subRS = rospy.Subscriber(topic_name, RoomStamped, self.callbackRoomStatus)
				topic_name = "/" + self.rooms_list[key_name]["name"] + "/magnetic_sensor_state"
				self.subMgn = rospy.Subscriber(topic_name, PointStamped, self.callbackMagneticSensor)

	def initPublishers(self):
		self.pubEmergency = rospy.Publisher("/emergency", EmergencyStamped, queue_size = 10)

	def gen_dictionary(self,name,variable):
		return {name:variable}

	def initVariables(self):
		self.rate = rospy.Rate(self.node_rate)
		self.kill_node = False
		self.room_status_data = {"data": [self.gen_dictionary(self.rooms_list["room" + str(i+1)]["name"],0) for i in range(len(self.rooms_list))]}
		self.magnetic_sensor_data = {"data": [self.gen_dictionary(self.rooms_list["room" + str(i+1)]["name"],0) for i in range(len(self.rooms_list))]}
		self.updatedRoomStatus = False

	def callbackKillNode(self,flag):
		self.kill_node = flag.data

	def callbackRoomStatus(self,room_status):
		self.room_status_data[room_status.header.frame_id] = room_status.status
		self.updatedRoomStatus = True
	
	def callbackMagneticSensor(self,magnetic):
		self.magnetic_sensor_data[magnetic.header.frame_id] = magnetic.point.x
		#print(magnetic.header.frame_id)
		#print(magnetic.point.x)
		self.updatedMagnetic = True

	def mainControl(self):
		rospy.loginfo("Orchestrator Monitoring: %s", self.active_rooms)
		while not rospy.is_shutdown():
			self.decision_algorithm()
			for i in range(len(self.active_rooms)):
				if (self.magnetic_sensor_data[self.active_rooms[i]) == 1) and (self.room_status_data[}self.active_rooms[i]] == 1)
				msg = EmergencyStamped()
				msg.header.stamp = rospy.Time.now()
				msg.emergency = 1
				self.pubEmergency.publish(msg)
				self.publishTopic = False
			if self.kill_node:
				break
			self.rate.sleep()
		rospy.loginfo("Orchestrator Finished")

if __name__ == '__main__':
	try:
		s = Orchestrator()
	except rospy.ROSInterruptException:
		pass
