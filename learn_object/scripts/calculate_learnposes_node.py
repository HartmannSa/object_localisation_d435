#!/usr/bin/env python
import yaml
import rospy
import tf2_ros
import tf2_geometry_msgs
import os
import errno
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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

def posesToDict(poses):
    poses_dict=dict()
    for elem in poses:
        pose_msg=poses[elem]
        pose=dict()
        pose["header"]=dict()
        pose["pose"]=dict()
        pose["pose"]["position"]=dict()
        pose["pose"]["orientation"]=dict()
        pose["header"]["frame_id"]=pose_msg.header.frame_id
        pose["pose"]["position"]["x"]=pose_msg.pose.position.x
        pose["pose"]["position"]["y"]=pose_msg.pose.position.y
        pose["pose"]["position"]["z"]=pose_msg.pose.position.z
        pose["pose"]["orientation"]["x"]=pose_msg.pose.orientation.x
        pose["pose"]["orientation"]["y"]=pose_msg.pose.orientation.y               
        pose["pose"]["orientation"]["z"]=pose_msg.pose.orientation.z
        pose["pose"]["orientation"]["w"]=pose_msg.pose.orientation.w
        poses_dict[elem]=pose
    return poses_dict

def save(poses, filename):
    if poses:
        # rospy.loginfo("Saving:")
        # rospy.loginfo(poses)
        # rospy.loginfo("to file "+filename)
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(filename, 'w') as outfile:
            yaml.safe_dump(posesToDict(poses),outfile,default_flow_style=False) 



if __name__=="__main__":
    rospy.init_node("learnObject_calculateLearnPoses")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(4.0)    
    
    # Get Parameter for Calculation
    numberImages        = rospy.get_param('~number_of_images', '10')
    angleRange          = rospy.get_param('~angle_range','180')
    filename_load       = rospy.get_param("~filename_load_pose",'')
    filename_save       = rospy.get_param("~filename_save_poses",'')
    filename_cam_save   = rospy.get_param("~filename_cam_save",'')
    distance            = rospy.get_param("~distance")
    source_frame        = rospy.get_param("~source_frame","base_link")
    target_frame        = rospy.get_param("~target_frame","map")

    # first_poses = dict()
    poses_base=dict()
    poses_cam = dict()
    angleRange_rad  = angleRange*math.pi/180
    
    # Load (first) Pose from .yaml
    # if os.path.isfile(filename_load):
    #     with open(filename_load, 'r') as infile:
    #         raw=yaml.safe_load(infile)
    #         if raw:
    #             first_poses=dictToPose(raw) 
    #     # print("Loaded pose: ")
    #     first_poses[1].header.frame_id = target_frame
    #     # print(first_poses)
    #     if len(first_poses) != 1:
    #         rospy.logerr("Just one first pose is needed, but %d are stored in %s. ", len(first_poses), filename_load)  
    # else:
    #     rospy.logerr("Could not load %s" %filename_load )    
    
    # Calculate learn poses
    angle = angleRange_rad/(numberImages-1)
    # first_pose = first_poses[1] 
    # print("Calculate with this first pose:")
    # print(first_pose)
    # first_rot = euler_from_quaternion([ first_pose.pose.orientation.x, first_pose.pose.orientation.y,first_pose.pose.orientation.z, first_pose.pose.orientation.w])
    # print(first_rot)
    
    cam_pose = PoseStamped()
    
    for i in range(0,numberImages):           
        # Calculate Cam Pose
        cam_pose.header.frame_id = "cam_footprint"
        cam_pose.pose.position.x = distance - distance*math.cos(i*angle)
        cam_pose.pose.position.y = distance*math.sin(i*angle)
        cam_pose.pose.position.z = 0.0
        cam_q = quaternion_from_euler(0, 0, -i*angle)
        cam_pose.pose.orientation = Quaternion(*cam_q)            

        succeeded = False
        while not succeeded:
            try:
                transform_cam = tfBuffer.lookup_transform("map", "cam_footprint", rospy.Time(), rospy.Duration(2.0))
                cam_pose_transformed = tf2_geometry_msgs.do_transform_pose(cam_pose, transform_cam)
                poses_cam[i+1]=cam_pose_transformed
                save(poses_cam, filename_cam_save)
                succeeded = True
            except:
                rate.sleep()
                continue

        # Calculate Base Pose
        base_pose = PoseStamped()
        base_pose.header.frame_id = poses_cam[i+1].header.frame_id
        base_pose.pose.position.x = poses_cam[i+1].pose.position.x - 0.2*math.sin(i*angle)
        base_pose.pose.position.y = poses_cam[i+1].pose.position.y - 0.2*math.cos(i*angle)
        base_pose.pose.position.z = 0
        base_pose.pose.orientation.x = poses_cam[i+1].pose.orientation.x
        base_pose.pose.orientation.y = poses_cam[i+1].pose.orientation.y
        base_pose.pose.orientation.z = poses_cam[i+1].pose.orientation.z
        base_pose.pose.orientation.w = poses_cam[i+1].pose.orientation.w
        poses_base[i+1] = base_pose
        save(poses_base, filename_save)

    print("Calculated Poses Done")
