import RPi.GPIO as GPIO
import time, sys, signal
import rospy
from std_msgs.msg import Bool

class Sensors():
	def __init__(self):
		self.rospy = rospy
		self.rospy.init_node('sensor_acquisition', anonymous = True)
		self.rospy.loginfo("Starting Sensors Acquisition")
        self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		#self.initServiceClients()
		self.initVariables()
		self.mainControl()

    def initParameters(self):
        self.room_name = self.rospy.get_param("~room_name","room1")
		self.read_magnetic = self.rospy.get_param("~magnetic_sensor/read",True)
        self.read_door_lasers = self.rospy.get_param("~door_laser_sensors/read",False)
        self.door_lasers_addresses = self.rospy.get_param("~door_laser_sensors/addresses",[0x29])
        self.door_lasers_names = self.rospy.get_param("~door_laser_sensors/names",["left"])
        self.node_rate = self.rospy.get_param("~node_rate",100)
        return

    def initSubscribers(self):
        self.subKillNode = self.rospy.Subscriber("/kill_sensors_acquisition", Bool, self.callbackKillNode)

    def initPublishers(self):
        if self.read_magnetic:
            self.rospy.loginfo("Acquiring Magnetic Sensor")
            topic_name = "/" self.room_name + "/magnetic_sensor_state"
            self.pubMgn = self.rospy.Publisher(topic_name, Bool, queue_size = 10)
        if self.read_door_laser:
            i2c_bus = subprocess.Popen(['i2cdetect', '-y', '1',], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = i2c_bus.communicate()
            devices_connected = stdout
            for i in range(length(self.door_lasers_addresses)):
                if (self.door_lasers_addressess[i] in devices_connected):
                    self.rospy.loginfo("Acquiring Magnetic Sensor '%s'",self.door_lasers_names[i])
                    topic_name = "/" + self.room_name + "/" + self.door_lasers_names[i] + "_laser_sensor_state"
                    self.pubDoorLaser[i] = self.rospy.Publisher(topic_name, Bool, queue_size = 10)

    def initVariables(self):
        self.rate = self.rospy.Rate(self.node_rate)
        self.magnetic_sensor_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.magnetic_sensor_pin, GPIO.IN)

    def callbackKillNode(self,flag):
        self.kill_node = flag.data

    def mainControl(self):
		self.rospy.loginfo("Sensors Acquisition in %s: OK", self.room_name)
        while not self.rospy.is_shutdown():
            self.pubMgn.publish(GPIO.input(self.magnetic_sensor_pin))
            if self.kill_node:
                break
            self.rate.sleep()
        self.rospy.loginfo("Sensors Acquisition in %s: Finished", self.room_name)

if __name__ == '__main__':
	try:
		s = Sensors()
	except rospy.ROSInterruptException:
		pass
