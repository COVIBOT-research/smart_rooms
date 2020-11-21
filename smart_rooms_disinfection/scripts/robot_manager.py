#!/usr/bin/python
import rospy
import numpy as np
import actionlib

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
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
        self.goals_topic = rospy.get_param("~goals_topic", "/current_goals")
        self.goals_status_topic = rospy.get_param("~goals_status_topic", "/goals_status")
        self.zone_topic = rospy.get_param("~zone_topic", "/disinfection_zone")
        self.rooms_list = rospy.get_param("~rooms_list", {})
        self.manager_rate = rospy.get_param("~rate", 50)
        self.wait_time = rospy.get_param("~wait_time", 5)
        self.retry_max = rospy.get_param("~maximum_retry", 3)
        self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
        self.param_lock = Lock()
        return

    def initSubscribers(self):
        self.sub_request = rospy.Subscriber(self.goals_topic, PoseArray, self.callbackPoses)
        return

    def initPublishers(self):
        self.pub_status = rospy.Publisher(self.goals_status_topic, String, queue_size = 10)
        self.pub_zone = rospy.Publisher(self.zone_topic, MarkerArray, queue_size = 10)
        self.pub_rooms = {}
        for i in range(len(self.rooms_list)):
			key_name = "room" + str(i+1)
			if self.rooms_list[key_name]["active"]:
				topic_name = "/" + self.rooms_list[key_name]["name"] + "/status"
				self.pub_ = rospy.Publisher(topic_name, String, queue_size)

        return

    def initServiceClients(self):
        self.service = rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
        return

    def initActionClients(self):
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        return

    def initVariables(self):
        self.msg_goal = MoveBaseGoal()
        self.msg_pose = Pose()
        self.count = 0
        self.marker_array = []
        self.retry_conn = 0
        self.change_goals = False
        self.final_goal_reached = True
        self.rate = rospy.Rate(self.manager_rate)
        return

    def callbackPoses(self, msg):
        self.frame_id = msg.header.frame_id
        self.goals = msg.poses
        self.goals_number = len(self.goals)
        self.goal_id = 0
        self.count = 0
        self.marker_array = []
        self.goal_published = False
        self.change_goals = True
        self.final_goal_reached = False
        return

    def callbackUpdateParams(self, req):
        with self.param_lock:
            self.initParameters()
            rospy.loginfo("[%s] Parameter update after request", self.name)
        return EmptyResponse()

    def drawZone(self):
        marker = Marker()
        marker.header.seq = self.count
        marker.header.stamp.secs = rospy.get_rostime().secs
        marker.header.stamp.nsecs = rospy.get_rostime().nsecs
        marker.header.frame_id = self.frame_id
        marker.id = self.count #Identifier
        marker.type = 3 #Cylinder
        marker.action = 0 #Add
        marker.pose.position.x = self.x_bot
        marker.pose.position.y = self.y_bot
        marker.scale.x = 4
        marker.scale.y = 4
        marker.scale.z = 0.1
        marker.color.a = 0.1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        if self.count % 10 == 0:
            self.marker_array.append(marker)
            print(len(self.marker_array))
            self.pub_zone.publish(self.marker_array)
        return

    def clearZone(self):
        self.pub_zone.publish([])
        return

    def getCurrentGoal(self):
        self.current_goal = self.goals[self.goal_id]
        self.x_goal, self.y_goal = self.current_goal.position.x, self.current_goal.position.y
        self.qz_goal, self.qw_goal = self.current_goal.orientation.z, self.current_goal.orientation.w
        return

    def makeMsgHeader(self):
        self.msg_goal.target_pose.header.frame_id = self.frame_id
        self.msg_goal.target_pose.header.stamp = rospy.Time.now()
        return

    def makeMsgPose(self):
        self.msg_pose.position.x = self.x_goal
        self.msg_pose.position.y = self.y_goal
        self.msg_pose.orientation.x = 0
        self.msg_pose.orientation.y = 0
        self.msg_pose.orientation.z = self.qz_goal
        self.msg_pose.orientation.w = self.qw_goal
        self.msg_goal.target_pose.pose = self.msg_pose
        return

    def publishGoal(self):
        self.getCurrentGoal()
        self.makeMsgHeader()
        self.makeMsgPose()
        self.action_client.send_goal(self.msg_goal, self.callbackDone, self.callbackActive, self.callbackFeedback)
        rospy.loginfo("[%s] Sending goald with ID %d to Action Server", self.name, self.goal_id)
        self.goal_published = True
        self.goal_reached = False
        return

    def callbackActive(self):
        rospy.loginfo("[%s] The goal with ID %d is now being processed by the Action Server...", self.name, self.goal_id)
        self.pub_status.publish("active")
        return

    def callbackFeedback(self, feedback):
        #print(feedback)
        self.x_bot = feedback.base_position.pose.position.x
        self.y_bot = feedback.base_position.pose.position.y
        if self.goal_id > 0 and not self.final_goal_reached:
            self.count += 1
            self.drawZone()
        d = np.sqrt(np.power(self.x_goal - self.x_bot, 2) + np.power(self.y_goal - self.y_bot, 2))
        if d < 0.5 and not self.final_goal_reached:
            rospy.loginfo("[%s] Goal with ID %s overriden", self.name, self.goal_id)
            self.callbackDone(3, "")
        return

    def callbackDone(self, status, result):
        if status == 2:
            rospy.loginfo("[%s] The goal with ID %d received a cancel request after it started executing", self.name, self.goal_id)
            self.pub_status.publish("cancelled")
        elif status == 3:
            rospy.loginfo("[%s] Reached Goal %d successfully", self.name, self.goal_id)
            self.goal_id += 1
            self.goal_published = False
            if self.goal_id > self.goals_number - 1:
                rospy.loginfo("[%s] Reached final goal", self.name)
                self.clearZone()
                self.final_goal_reached = True
                self.pub_status.publish("finished")
                return
        elif status == 4:
            rospy.loginfo("[%s] The goal with ID %d was aborted by the Action Server", self.name, self.goal_id)
            #rospy.signal_shutdown("Goal with ID "+str(self.goal_id)+" aborted, shutting down!")
            return
        elif status == 5:
            rospy.loginfo("[%s] The goal with ID %d has been rejected by the Action Server", self.name, self.goal_id)
            #rospy.signal_shutdown("Goal with ID "+str(self.goal_id)+" rejected, shutting down!")
            return
        elif status == 8:
            rospy.loginfo("[%s] The goal with ID %d received a cancel request before it started executing, successfully cancelled!", self.name, self.goal_id)
        else:
            pass
        return

    def main(self):
        #We wait for the move base server during 5 seconds
        self.wait = self.action_client.wait_for_server(rospy.Duration(self.wait_time))
        if self.wait and self.retry_conn < self.retry_max:
            rospy.loginfo("[%s] Configuration OK", self.name)
            rospy.loginfo("[%s] Connected to move base server", self.name)
            self.pub_status.publish("prepared")
            while not rospy.is_shutdown():
                if not self.final_goal_reached and self.change_goals:
                    if not self.goal_published:
                        self.publishGoal()
                else:
                    self.change_goals = False
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
