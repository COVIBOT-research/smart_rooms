#!/usr/bin/python
import rospy
import numpy as np
import actionlib

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from smart_rooms_msgs.msg import RoomStamped, RoomStampedList
from std_srvs.srv import EmptyResponse, Empty
from threading import Lock

class RobotManager():
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous = True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initServiceClients()
        self.initActionClients()
        self.initVariables()
        self.main()

    def initParameters(self):
        self.requestTopic = rospy.get_param("~disinfection/request_topic", "/disinfection/request")
        self.managerRate = rospy.get_param("~rate", 50)
        self.wait_time = rospy.get_param("~wait_time", 5)
        self.retry_max = rospy.get_param("~maximum_retry", 3)
        self.updateParamsService = self.name + rospy.get_param("~update_params_service", "/update_parameters")
        self.param_lock = Lock()
        return

    def initSubscribers(self):
        self.sub_request = rospy.Subscriber(self.requestTopic, RoomStampedList, self.callbackRequest)
        return

    def initPublishers(self):
        pass

    def initServiceClients(self):
        self.service = rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
        return

    def initActionClients(self):
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        return

    def initVariables(self):
        self.retry_conn = 0
        self.change_request = True
        self.rate = rospy.Rate(self.managerRate)
        return

    def callbackRequest(self, msg):
        self.requests_cue = msg
        self.change_request = True
        return

    def callbackUpdateParams(self, req):
        with self.param_lock:
            self.initParameters()
            rospy.loginfo("[%s] Parameter update after request", self.name)
        return EmptyResponse()

    def main(self):
        self.wait = self.action_client.wait_for_server(rospy.Duration(self.wait_time))
        if self.wait and self.retry_conn < self.retry_max:
            rospy.loginfo("[%s] Configuration OK", self.name)
            rospy.loginfo("[%s] Connected to move base server", self.name)
            while not rospy.is_shutdown():

                self.rate.sleep()
        else:
            if self.retry_conn < 3:
                rospy.logwarn("[%s] Action server did not respond after %d seconds", self.name, self.wait_time)
                rospy.logwarn("[%s] Retrying to reach connection ...", self.name)
                self.retry_conn += 1
                self.main()
            else:
                rospy.logerr("[%s] Unable to reach connection ...", self.name)
        return

if __name__ == '__main__':
    try:
        sw = RobotManager('robot_manager')
    except Exception as e:
        print("Something bad happened: ")
        print(e)
