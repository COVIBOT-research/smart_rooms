#!/usr/bin/python
from datetime import datetime
import rospy
from std_msgs.msg import Bool, String, Int16
from geometry_msgs.msg import PointStamped

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
		self.class_hours = rospy.get_param("~schedules/class","[8.5, 10, 13, 16]")
		self.cleaning_hours = rospy.get_param("~schedules/cleaning,","[9, 12, 15, 18]")
		self.node_rate = int(rospy.get_param("~node_paramters/rate","100"))
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
		self.pubDisinfectionRequest = rospy.Publisher("/disinfection/request", PointStamped, queue_size = 10)

	def gen_dictionary(self,name,variable):
		return {name:variable}

	def initVariables(self):
		self.rate = rospy.Rate(self.node_rate)
		self.kill_node = False
		self.magnetic_sensor_data = {"data": [self.gen_dictionary(self.rooms_list["room" + str(i+1)]["name"],0) for i in range(len(self.rooms_list))]}
		self.people_number = {"data": [self.gen_dictionary(self.rooms_list["room" + str(i+1)]["name"],0) for i in range(len(self.rooms_list))]}
		self.updatedMagnetic = False
		self.updatedPeople = False
		self.sent_request = []
		''' Decision Algorithm Parameters '''
		self.w_time = 0.5
		self.w_np = 0.5
		self.p_threshold = 50

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

	def decision_algorithm(self):
		if (self.updatedMagnetic == True) or (self.updatedPeople == True):
			for i in range(len(self.active_rooms)):
				try:
					if (self.magnetic_sensor_data[self.active_rooms[i]] == 1):
						if not (self.active_rooms[i] in self.sent_request):
							''' Calculate time between disinfection schedules and current time'''
							#print("NO ESTA ---------------------------------")
							now = datetime.now()
							current_date = now.strftime("%Y-%m-%d")
							cleaning_times = []
							for j in range(len(self.rooms_list[self.active_rooms[i]]["schedules"]["cleaning"])):
								cleaning_time = datetime.strptime(current_date + " " + self.rooms_list[self.active_rooms[i]]["schedules"]["cleaning"][j],"%Y-%m-%d %H:%M:%S")
								time = cleaning_time - now
								if time.days == 0:
									time_minutes = time.seconds/60
									p = self.magnetic_sensor_data[self.active_rooms[i]]*(self.w_time*time_minutes + self.w_np*self.people_number[self.active_rooms[i]])
									print(p)
									if p > self.p_threshold:
										msg = PointStamped()
										msg.header.stamp = rospy.Time.now()
										msg.header.frame_id = self.active_rooms[i]
										msg.point.z = 1
										msg.point.y = p
										self.pubDisinfectionRequest.publish(msg)
										rospy.loginfo("Requesting Disinfection for Room: %s",self.active_rooms[i])
										self.sent_request.append(self.active_rooms[i])
										print(self.sent_request)
									break
							self.updatedMagnetic = False
							self.updatedPeople = False
							return
						else:
							pass
					else:
						if self.active_rooms[i] in self.sent_request:
							msg = PointStamped()
							msg.header.stamp = rospy.Time.now()
							msg.header.frame_id = self.active_rooms[i]
							msg.point.z = 0
							msg.point.y = 0
							msg.point.x = 0
							self.pubDisinfectionRequest.publish(msg)
							self.sent_request.remove(self.active_rooms[i])
							print(self.sent_request)
							rospy.loginfo("Cancelling Disinfection for Room: %s",self.active_rooms[i])
						return
				except:
					return
		else:
			return

	def mainControl(self):
		rospy.loginfo("Orchestrator Monitoring: %s", self.active_rooms)
		while not rospy.is_shutdown():
			self.decision_algorithm()
			if self.kill_node:
				break
			self.rate.sleep()
		rospy.loginfo("Orchestrator Finished")

if __name__ == '__main__':
	try:
		s = Orchestrator()
	except rospy.ROSInterruptException:
		pass
