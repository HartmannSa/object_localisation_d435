#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, PointStamped
from visualization_msgs.msg import Marker


class TargetMarker:

    def __init__(self):        
        self.__targetMarker     = Marker()        
        self.__topicSub          = rospy.get_param("~topic_sub")
        self.__topicPub          = rospy.get_param("~topic_pub")
        self.__dimX             = rospy.get_param("~dimX", '0.4')
        self.__dimY             = rospy.get_param("~dimY", '0.3')
        self.__dimZ             = rospy.get_param("~dimZ", '0.3')
        self.__pose_sub         = rospy.Subscriber(self.__topicSub, PoseStamped, self.__setMarker__)
        self.__marker_pub       = rospy.Publisher(self.__topicPub, Marker, queue_size=10)
    
    
    def __setMarker__(self, msg):        
        self.__targetMarker.header.frame_id = msg.header.frame_id
        self.__targetMarker.pose.position = msg.pose.position
        self.__targetMarker.pose.position.z = self.__dimZ/2
        self.__targetMarker.pose.orientation = msg.pose.orientation
        self.__targetMarker.id = 0
        self.__targetMarker.type = 1       # 1: cube 2: sphere
        self.__targetMarker.action = 0

        self.__targetMarker.scale.x = self.__dimX
        self.__targetMarker.scale.y = self.__dimY
        self.__targetMarker.scale.z = self.__dimZ
        self.__targetMarker.color.r = 0.0
        self.__targetMarker.color.g = 0.0
        self.__targetMarker.color.b = 0.0
        self.__targetMarker.color.a = 0.5
        self.__targetMarker.lifetime = rospy.Duration(0)
    
    def __publishMarker__(self):
        self.__marker_pub.publish(self.__targetMarker)
    

if __name__=="__main__":
    rospy.init_node("learnObject_spawnMarker", anonymous=True)
    mark = TargetMarker()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():        
        mark.__publishMarker__()
        rate.sleep()

