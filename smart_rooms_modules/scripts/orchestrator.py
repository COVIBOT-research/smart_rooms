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
		self.class_hours = rospy.get_param("~schedules/class")
		self.cleaning_hours = rospy.get_param("~schedules/cleaning")
		self.node_rate = int(rospy.get_param("~node_paramters/rate","100"))
		self.trajectory = rospy.get_param("~trial_parameters/trajectory","spiral")
		self.disinfection_method = rospy.get_param("~trial_parameters/disinfection_method","uv")
		self.rooms_list = rospy.get_param("rooms_list")
		return

	def initSubscribers(self):
		self.subKillNode = rospy.Subscriber("/kill_orchestrator", Bool, self.callbackKillNode)
		self.active_rooms = []
		for i in range(len(self.rooms_list)):
			key_name = "room" + str(i+1)
			if self.rooms_list[key_name]["active"]:
				self.active_rooms.append(key_name)
				topic_name = "/" + self.rooms_list[key_name]["name"] + "/magnetic_sensor_state"
				self.subMgn = rospy.Subscriber(topic_name, PointStamped, self.callbackMagneticSensor)
				topic_name = "/" + self.rooms_list[key_name]["name"] + "/people_number"
				self.subPC = rospy.Subscriber(topic_name, PointStamped, self.callbackPeopleNumber)

	def initPublishers(self):
		self.pubDisinfectionRequest = rospy.Publisher("/disinfection/request", RoomStampedList, queue_size = 10)

	def gen_dictionary(self,name,variable):
		return {name:variable}

	def initVariables(self):
		self.rate = rospy.Rate(self.node_rate)
		self.kill_node = False
		self.magnetic_sensor_data = {"data": [self.gen_dictionary(self.rooms_list["room" + str(i+1)]["name"],0) for i in range(len(self.rooms_list))]}
		self.people_number = {"data": [self.gen_dictionary(self.rooms_list["room" + str(i+1)]["name"],0) for i in range(len(self.rooms_list))]}
		self.updatedMagnetic = False
		self.updatedPeople = False
		self.publishTopic = False
		self.sent_request_rooms = []
		self.sent_request_p = []
		self.optimized_request_rooms = []
		''' Decision Algorithm Parameters '''
		self.w_time = 0.5
		self.w_np = 0.5
		self.p_threshold = 15

	def callbackKillNode(self,flag):
		self.kill_node = flag.data

	def callbackMagneticSensor(self,magnetic):
		self.magnetic_sensor_data[magnetic.header.frame_id] = magnetic.point.x
		#print(magnetic.header.frame_id)
		#print(magnetic.point.x)
		self.updatedMagnetic = True

	def callbackPeopleNumber(self,number):
		self.people_number[number.header.frame_id] = number.point.x
		self.updatedPeople = True

	def optimize_request(self):
		sorted_p = list(self.sent_request_p)
		sorted_p.sort(reverse=True)
		optimized_request_rooms = []
		for i in range(len(sorted_p)):
			optimized_request_rooms.append(self.sent_request_rooms[self.sent_request_p.index(sorted_p[i])])
		self.sent_request_rooms = optimized_request_rooms
		self.sent_request_p = sorted_p

	def decision_algorithm(self):
		if (self.updatedMagnetic == True) or (self.updatedPeople == True):
			for i in range(len(self.active_rooms)):
				try:
					if (self.magnetic_sensor_data[self.active_rooms[i]] == 1):
						if not (self.active_rooms[i] in self.sent_request_rooms):
							''' Calculate time between disinfection schedules and current time'''
							now = datetime.now()
							current_date = now.strftime("%Y-%m-%d")
							cleaning_times = []
							for j in range(len(self.rooms_list[self.active_rooms[i]]["schedules"]["cleaning"])):
								cleaning_time = datetime.strptime(current_date + " " + self.rooms_list[self.active_rooms[i]]["schedules"]["cleaning"][j],"%Y-%m-%d %H:%M:%S")
								time = cleaning_time - now
								#print(time.days)
								if time.days == 0:
									time_minutes = time.seconds/60
									p = (self.w_time*time_minutes) + (self.w_np*self.people_number[self.active_rooms[i]])
									if p > self.p_threshold:
										#msg = PointStamped()
										#msg.header.stamp = rospy.Time.now()
										#msg.header.frame_id = self.active_rooms[i]
										#msg.point.z = 1
										#msg.point.y = p
										#self.pubDisinfectionRequest.publish(msg)
										rospy.loginfo("Requesting Disinfection for Room: %s",self.active_rooms[i])
										self.sent_request_rooms.append(self.active_rooms[i])
										self.sent_request_p.append(p)
										self.optimize_request()
										self.publishTopic = True
										#print(self.sent_request_rooms)
										#print(self.sent_request_p)
									break
							self.updatedMagnetic = False
							self.updatedPeople = False
							return
						else:
							pass
					else:
						if self.active_rooms[i] in self.sent_request_rooms:
							#msg = PointStamped()
							#msg.header.stamp = rospy.Time.now()
							#msg.header.frame_id = self.active_rooms[i]
							#msg.point.z = 0
							#msg.point.y = 0
							#msg.point.x = 0
							#self.pubDisinfectionRequest.publish(msg)
							self.sent_request_p.pop(self.sent_request_rooms.index(self.active_rooms[i]))
							self.sent_request_rooms.remove(self.active_rooms[i])
							rospy.loginfo("Cancelling Disinfection for Room: %s",self.active_rooms[i])
							#print(self.sent_request_rooms)
							#print(self.sent_request_p)
						pass
				except Exception as e:
					pass
		else:
			return

	def mainControl(self):
		rospy.loginfo("Orchestrator Monitoring: %s", self.active_rooms)
		while not rospy.is_shutdown():
			self.decision_algorithm()
			if ((len(self.sent_request_rooms) != 0) and (self.publishTopic == True)):
				msg_list = []
				for i in range(len(self.sent_request_rooms)):
					msg = RoomStamped()
					msg.header.stamp = rospy.Time.now()
					msg.header.frame_id = self.sent_request_rooms[i]
					msg.room_name = self.rooms_list[self.sent_request_rooms[i]]["name"]
					msg.trajectory = self.trajectory
					msg.disinfection_method = self.disinfection_method
					print(msg)
					msg_list.append(msg)
				self.pubDisinfectionRequest.publish(msg_list)
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
