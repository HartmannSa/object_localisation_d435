#!/usr/bin/env python
import yaml
import rospy
import os
import errno
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import actionlib
from learn_object.msg import cameraAction, cameraGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from learn_object.srv import saveFrame,saveFrameResponse
import tf2_ros
from std_msgs.msg import String


def dictToPose(poses):
    pose_list=dict()
    for elem in poses:
        pose=poses[elem]
        pose_msg=PoseStamped()
        pose_msg.header.frame_id=pose["header"]["frame_id"]
        pose_msg.pose.position.x=pose["pose"]["position"]["x"]
        pose_msg.pose.position.y=pose["pose"]["position"]["y"]
        pose_msg.pose.position.z=pose["pose"]["position"]["z"]
        pose_msg.pose.orientation.x=pose["pose"]["orientation"]["x"]
        pose_msg.pose.orientation.y=pose["pose"]["orientation"]["y"]
        pose_msg.pose.orientation.z=pose["pose"]["orientation"]["z"]
        pose_msg.pose.orientation.w=pose["pose"]["orientation"]["w"]
        pose_list[elem]=pose_msg
    return pose_list


class PosePublisher:

    def __init__(self):
        self.__pose_sub = rospy.Subscriber("/calculation_done", String, self.__refresh__)
        self.__poses = dict()
        self.__filename  = rospy.get_param('~filename', '')
        self.__topic     = rospy.get_param('~topic', '')
        self.__pub_1 = rospy.Publisher(self.__topic + "/pose_1", PoseStamped, queue_size=2)
        self.__pub_2 = rospy.Publisher(self.__topic + "/pose_2", PoseStamped, queue_size=2)
        self.__pub_3 = rospy.Publisher(self.__topic + "/pose_3", PoseStamped, queue_size=2)
        self.__pub_4 = rospy.Publisher(self.__topic + "/pose_4", PoseStamped, queue_size=2)
        self.__pub_5 = rospy.Publisher(self.__topic + "/pose_5", PoseStamped, queue_size=2)
        self.__pub_6 = rospy.Publisher(self.__topic + "/pose_6", PoseStamped, queue_size=2)
        self.__pub_7 = rospy.Publisher(self.__topic + "/pose_7", PoseStamped, queue_size=2)
        self.__pub_8 = rospy.Publisher(self.__topic + "/pose_8", PoseStamped, queue_size=2)
        self.__pub_9 = rospy.Publisher(self.__topic + "/pose_9", PoseStamped, queue_size=2)
        self.__pub_10 = rospy.Publisher(self.__topic + "/pose_10", PoseStamped, queue_size=2)
        self.__pub_11 = rospy.Publisher(self.__topic + "/pose_11", PoseStamped, queue_size=2)
        self.__pub_12 = rospy.Publisher(self.__topic + "/pose_12", PoseStamped, queue_size=2)
        self.__loadPoses__()
    
    def __loadPoses__(self):
        if os.path.isfile(self.__filename):
            with open(self.__filename, 'r') as infile:
                raw=yaml.safe_load(infile)
                if raw:
                    self.__poses=dictToPose(raw)
                    print("Loaded poses")
        else:
            rospy.logerr("Could not load %s" %self.__filename )

        
    def __refresh__(self, msg):
        self.__loadPoses__()
    
    def publishPoses(self):
        self.__pub_1.publish(self.__poses[1])
        self.__pub_2.publish(self.__poses[2])
        self.__pub_3.publish(self.__poses[3])
        self.__pub_4.publish(self.__poses[4])
        self.__pub_5.publish(self.__poses[5])
        self.__pub_6.publish(self.__poses[6])
        self.__pub_7.publish(self.__poses[7])
        self.__pub_8.publish(self.__poses[8])
        self.__pub_9.publish(self.__poses[9])
        self.__pub_10.publish(self.__poses[10])
        self.__pub_11.publish(self.__poses[11])
        self.__pub_12.publish(self.__poses[12])


if __name__=="__main__":
    rospy.init_node("learnObject_publishPoses", anonymous=True)
    pub = PosePublisher()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publishPoses()
        rate.sleep()


