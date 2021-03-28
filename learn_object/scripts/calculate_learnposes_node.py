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
from tf.transformations import quaternion_from_euler

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
    print(poses_dict)        
    return poses_dict

def save(poses, filename):
    if poses:
        rospy.loginfo("Saving:")
        rospy.loginfo(poses)
        rospy.loginfo("to file "+filename)

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
    numberImages    = rospy.get_param('~number_of_images', '10')
    angleRange      = rospy.get_param('~angle_range','180')
    filename_load   = rospy.get_param("~filename_load_pose",'')
    filename_save   = rospy.get_param("~filename_save_poses",'')
    distance        = rospy.get_param("~distance")
    source_frame    = rospy.get_param("~source_frame","base_link")
    target_frame    = rospy.get_param("~target_frame","map")

    poses=dict()
    angleRange_rad  = angleRange*math.pi/180
    
    # Load (first) Pose from .yaml
    if os.path.isfile(filename_load):
        with open(filename_load, 'r') as infile:
            raw=yaml.safe_load(infile)
            if raw:
                poses=dictToPose(raw)  
        print("Loaded pose: ")
        print(poses)
        if len(poses) != 1:
            rospy.logerr("Just one first pose is needed, but %d are stored in %s. ", len(poses), filename_load)  
    else:
        rospy.logerr("Could not load %s" %filename_load )    
    
    # Calculate learn poses
    angle = angleRange_rad/numberImages
    current_pose = PoseStamped()
    for i in range(1,numberImages): # 0 bis 9        
        current_pose.header.frame_id = source_frame
        current_pose.pose.position.x = -distance*math.cos(i*angle)
        current_pose.pose.position.y = distance*math.sin(i*angle)
        current_pose.pose.position.z = 0.0
        q =quaternion_from_euler(0, 0, i*angle)
        current_pose.pose.orientation = Quaternion(*q)
        # current_pose.pose.orientation = quaternion_about_axis(i*angle, (0,0,1))
        # current_pose.pose.orientation.x = 0
        # current_pose.pose.orientation.y = 0
        # current_pose.pose.orientation.z = 0
        # current_pose.pose.orientation.w = 1
        
        succeeded = False
        while not succeeded:
            try:
                transform = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(2.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(current_pose, transform)
                poses[len(poses)+1]=pose_transformed
                save(poses, filename_save)
                succeeded = True
            except:
                rate.sleep()
                continue

    # Save poses in new file

    
    print("Calculated Poses")
    #rospy.spin()
