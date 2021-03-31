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


if __name__=="__main__":
    rospy.init_node("learnObject_publishPoses")

    # Get Parameter
    filename_poses        = rospy.get_param('~filename_poses', '')
    filename_poses_cam    = rospy.get_param('~filename_cam_poses', '')
    poses=dict()
    poses_cam = dict()

    # Load Base Poses from .yaml
    if os.path.isfile(filename_poses):
        with open(filename_poses, 'r') as infile:
            raw=yaml.safe_load(infile)
            if raw:
                poses=dictToPose(raw)
    else:
        rospy.logerr("Could not load %s" %filename_poses )

    # Load Cam Poses from .yaml
    if os.path.isfile(filename_poses_cam):
        with open(filename_poses_cam, 'r') as infile:
            raw=yaml.safe_load(infile)
            if raw:
                poses_cam=dictToPose(raw)
    else:
        rospy.logerr("Could not load %s" %filename_poses_cam )

    pub_base_1 = rospy.Publisher("/base/pose_1", PoseStamped, queue_size=2)
    pub_base_2 = rospy.Publisher("/base/pose_2", PoseStamped, queue_size=2)
    pub_base_3 = rospy.Publisher("/base/pose_3", PoseStamped, queue_size=2)
    pub_cam_1 = rospy.Publisher("/cam/pose_1", PoseStamped, queue_size=2)
    pub_cam_2 = rospy.Publisher("/cam/pose_2", PoseStamped, queue_size=2)
    pub_cam_3 = rospy.Publisher("/cam/pose_3", PoseStamped, queue_size=2)

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        pub_base_1.publish(poses[1])
        pub_base_2.publish(poses[2])
        pub_base_3.publish(poses[3])
        pub_cam_1.publish(poses_cam[1])
        pub_cam_2.publish(poses_cam[2])
        pub_cam_3.publish(poses_cam[3])      
        rate.sleep()


