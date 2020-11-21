#!/usr/bin/python
import rospy
import numpy as np
import actionlib
import csv
from sys import stdin
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from smart_rooms_msgs.msg import RoomStamped, RoomStampedList
from std_srvs.srv import EmptyResponse, Empty
from threading import Lock

from tf.transformations import quaternion_from_euler as qfe

class RequestManager():
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous = True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initServiceClients()
        self.initVariables()
        self.main()

    def initParameters(self):
        self.request_topic = rospy.get_param("~request_topic", "/disinfection/request")
        self.goals_topic = rospy.get_param("~goals_topic", "/current_goals")
        self.goals_status_topic = rospy.get_param("~goals_status_topic", "/goals_status")
        self.parent_path = rospy.get_param("~parent_path", "/home/walker/catkin_ws/src/smart_rooms/smart_rooms_disinfection/config/")
        self.rooms_list = rospy.get_param("~rooms_list", {})
        self.trajectories = rospy.get_param("~trajectories", [])
        self.manager_rate = rospy.get_param("~rate", 50)
        self.updateParamsService = self.name + rospy.get_param("~update_params_service", "/update_parameters")
        self.param_lock = Lock()
        return

    def initSubscribers(self):
        self.sub_request = rospy.Subscriber(self.request_topic, RoomStampedList, self.callbackRequest)
        self.sub_goals_status = rospy.Subscriber(self.goals_status_topic, String, self.callbackGoalStatus)
        return

    def initPublishers(self):
        self.pub_goals = rospy.Publisher(self.goals_topic, PoseArray, queue_size = 10)
        pass

    def initServiceClients(self):
        self.service = rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
        return

    def initVariables(self):
        self.last_request = RoomStamped()
        self.robot_is_prepared = False
        self.robot_is_disinfecting = False
        self.request_cue = False
        self.rate = rospy.Rate(self.manager_rate)
        return

    def callbackUpdateParams(self, req):
        with self.param_lock:
            self.initParameters()
            rospy.loginfo("[%s] Parameter update after request", self.name)
        return EmptyResponse()

    def callbackRequest(self, msg):
        self.requests = msg.rooms
        self.request_cue = True
        self.request_id = 0
        return

    def callbackGoalStatus(self, msg):
        if msg.data == "prepared":
            rospy.loginfo("[%s] Received confirmation: Robot is prepared", self.name)
            self.robot_is_prepared = True
        elif msg.data == "active":
            self.robot_is_disinfecting = True
        elif msg.data == "finished":
            rospy.loginfo("[%s] Received confirmation: Robot finished disinfection", self.name)
            self.requests = self.requests[1::]
            self.robot_is_disinfecting = False
            self.request_id += 1
        else:
            pass
        return

    def read_file(self, file):
        try:
            with open(file) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for line_counter, row in enumerate(csv_reader):
                    if line_counter > 0:
                        self.frame_id = row[0] #We just need one
                        row = row[1:3]
                        row_i = np.reshape(map(float, np.array(row)), (1, 2))
                        if line_counter == 1:
                            self.poses = row_i
                        else:
                            self.poses = np.concatenate((self.poses, row_i))
                self.poses_number = self.poses.size
            self.read_flag = True
        except Exception as e:
            print(e)
            self.read_flag = False
        return

    def fix_poses(self, trajectory):
        trajectories_info = self.trajectories.get(self.room_id, {})
        trajectory_goals = trajectories_info.get(trajectory, [])
        self.final_poses = []
        print(len(self.poses[0]))
        print(len(self.poses))
        for i in range(len(trajectory_goals)):
            pose = []
            if i < len(trajectory_goals) - 1:
                idx_sig = trajectory_goals[i+1]-1
            else:
                idx_sig = trajectory_goals[i-1]-1
            idx = trajectory_goals[i]-1
            x = self.poses[idx][0]
            y = self.poses[idx][1]
            x_sig = self.poses[idx_sig][0]
            y_sig = self.poses[idx_sig][1]
            theta = np.arctan2(y_sig - y, x_sig - x)
            quat = qfe(0, 0, theta) #roll, pitch, yaw
            pose = [round(x, 2), round(y, 2), round(quat[2], 2), round(quat[3], 2)]
            print(pose)
            self.final_poses.append(pose)
        return

    def makeMsgGoalArray(self):
        self.msg_pose_array = PoseArray()
        self.msg_pose_array.header.frame_id = self.frame_id
        self.msg_pose_array.header.stamp = rospy.Time.now()
        array = []
        for pose in self.final_poses:
            msg_pose = Pose()
            msg_pose.position.x = pose[0]
            msg_pose.position.y = pose[1]
            msg_pose.orientation.x = 0
            msg_pose.orientation.y = 0
            msg_pose.orientation.z = pose[2]
            msg_pose.orientation.w = pose[3]
            array.append(msg_pose)
        self.msg_pose_array.poses = array
        return

    def getCurrentRequest(self):
        self.current_request = self.requests[self.request_id]
        print(self.current_request)
        if self.current_request.header.frame_id != self.last_request.header.frame_id:
            #Process it
            self.room_id = self.current_request.header.frame_id
            self.room_info = self.rooms_list.get(self.room_id, {})
            self.poses_file = self.room_info["poses_file"]
            #We read the .csv file according to the parameters (e.g. room1.csv)
            self.read_file(self.parent_path+self.poses_file)
            #We fix the poses, estimating quaternions and the poses to be used
            self.fix_poses(self.current_request.trajectory)
            self.makeMsgGoalArray()
            self.pub_goals.publish(self.msg_pose_array)
        else:
            #Ignore it, because its the sameone
            pass
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            if self.request_cue and len(self.requests) > 0:
                if not self.robot_is_disinfecting:
                    self.getCurrentRequest()
            self.rate.sleep()

        return

if __name__ == '__main__':
    #try:
    sw = RequestManager('request_manager')
    #except Exception as e:
    #    print("Something bad happened: ")
    #    print(e)
