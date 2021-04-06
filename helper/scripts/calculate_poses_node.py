#!/usr/bin/env python
import yaml
import rospy
import tf2_ros
import tf2_geometry_msgs
import os
import errno
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, PointStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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

class PoseCalculator:

    def __init__(self):
        self.__numberPoses      = rospy.get_param('~number_of_poses', '10')
        self.__filePath         = rospy.get_param("~file_path",'')  
        self.__linear           = rospy.get_param("~linear", True)              
        self.__stepDistance     = rospy.get_param("~stepDistance", '0.2')
        self.__stepOffset       = rospy.get_param("~stepOffset", '0.5')
        self.__circular         = rospy.get_param("~circular", True)        
        self.__radius           = rospy.get_param("~radius", '1.0')
        self.__angleRange       = rospy.get_param('~angle_range','180')
        self.__dx               = rospy.get_param('~dx','0.3')
        self.__dy               = rospy.get_param('~dy','0.2')
        self.__pose_sub         = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.__calcPoints__)
        self.__calc_pub         = rospy.Publisher("/calculation_done", String, queue_size=2)   
    
    
    def __calcPoints__(self, msg):
        if self.__linear:
            self.__calcPointsLinear__(msg)
        if self.__circular:
            self.__calcPointsCircular__(msg)

    def __calcPointsCircular__(self, msg):
        transform_mapTarget = geometry_msgs.msg.TransformStamped()
        transform_mapTarget.header.frame_id = "map"
        transform_mapTarget.child_frame_id = "targetFrame"
        transform_mapTarget.transform.translation.x = msg.pose.position.x
        transform_mapTarget.transform.translation.y = msg.pose.position.y
        transform_mapTarget.transform.translation.z = msg.pose.position.z
        transform_mapTarget.transform.rotation.x = msg.pose.orientation.x
        transform_mapTarget.transform.rotation.y = msg.pose.orientation.y
        transform_mapTarget.transform.rotation.z = msg.pose.orientation.z
        transform_mapTarget.transform.rotation.w = msg.pose.orientation.w

        off_point = geometry_msgs.msg.PointStamped()
        off_point.header.frame_id = "targetFrameI"
        off_point.point.x = self.__radius + self.__dx
        off_point.point.y = self.__dy
        off_point.point.z = 0

        transform_targetI = geometry_msgs.msg.TransformStamped()
        transform_targetI.header.frame_id = "targetFrame"
        transform_targetI.child_frame_id = "targetFrameI"
        transform_targetI.transform.translation.x = 0
        transform_targetI.transform.translation.y = 0
        transform_targetI.transform.translation.z = 0

        angleRange_rad  = self.__angleRange*math.pi/180
        angleStep = angleRange_rad/(self.__numberPoses-1)

        poses = dict()
        poses_base  = dict()
        pose_i = PoseStamped()
        
        pose_i.header.frame_id = "targetFrame"
        pose_i.pose.position.z = 0
        for i in range(0, self.__numberPoses):
            # Cam pose
            pose_i.pose.position.x = self.__radius*math.cos(i*angleStep)
            pose_i.pose.position.y = self.__radius*math.sin(i*angleStep)
            q = quaternion_from_euler(0, 0, math.pi+i*angleStep)
            pose_i.pose.orientation.x = q[0]
            pose_i.pose.orientation.y = q[1]
            pose_i.pose.orientation.z = q[2]
            pose_i.pose.orientation.w = q[3]
            pose_i_transformed = tf2_geometry_msgs.do_transform_pose(pose_i, transform_mapTarget)
            poses[i+1]=pose_i_transformed

            # Calculate Offset (Base) Pose
            q_i = quaternion_from_euler(0, 0, i*angleStep)
            transform_targetI.transform.rotation.x = q_i[0]
            transform_targetI.transform.rotation.y = q_i[1]
            transform_targetI.transform.rotation.z = q_i[2]
            transform_targetI.transform.rotation.w = q_i[3] 
            
            base_point_T = tf2_geometry_msgs.do_transform_point(off_point, transform_targetI)
            base_point = tf2_geometry_msgs.do_transform_point(base_point_T, transform_mapTarget)
            base_pose = PoseStamped()
            base_pose.header.frame_id = poses[i+1].header.frame_id
            base_pose.pose.position.x = base_point.point.x
            base_pose.pose.position.y = base_point.point.y
            base_pose.pose.position.z = 0
            base_pose.pose.orientation.x = poses[i+1].pose.orientation.x
            base_pose.pose.orientation.y = poses[i+1].pose.orientation.y
            base_pose.pose.orientation.z = poses[i+1].pose.orientation.z
            base_pose.pose.orientation.w = poses[i+1].pose.orientation.w
            poses_base[i+1] = base_pose

        save(poses_base, self.__filePath + "poses_circular_base.yaml")        
        save(poses, self.__filePath + "poses_circular.yaml")
        rospy.loginfo("Saved circular calculated poses")
        self.__calc_pub.publish("Done circular calculation")

    def __calcPointsLinear__(self, msg):
        poses = dict()
        pose_i = PoseStamped()
        transform_mapTarget = geometry_msgs.msg.TransformStamped()

        transform_mapTarget.header.frame_id = "map"
        transform_mapTarget.child_frame_id = "targetFrame"
        transform_mapTarget.transform.translation.x = msg.pose.position.x
        transform_mapTarget.transform.translation.y = msg.pose.position.y
        transform_mapTarget.transform.translation.z = msg.pose.position.z
        transform_mapTarget.transform.rotation.x = msg.pose.orientation.x
        transform_mapTarget.transform.rotation.y = msg.pose.orientation.y
        transform_mapTarget.transform.rotation.z = msg.pose.orientation.z
        transform_mapTarget.transform.rotation.w = msg.pose.orientation.w

        pose_i.header.frame_id = "targetFrame"
        pose_i.pose.position.y = 0
        pose_i.pose.position.z = 0
        pose_i.pose.orientation.x = 0
        pose_i.pose.orientation.y = 0
        pose_i.pose.orientation.z = 0
        pose_i.pose.orientation.w = 1.0

        for i in range(1, self.__numberPoses+1):
            pose_i.pose.position.x = -self.__stepOffset - i*self.__stepDistance
            pose_i_transformed = tf2_geometry_msgs.do_transform_pose(pose_i, transform_mapTarget)
            poses[i]=pose_i_transformed

        save(poses, self.__filePath + "poses_linear.yaml")
        rospy.loginfo("Saved linear calculated poses")
        self.__calc_pub.publish("Done linear calculation")


if __name__=="__main__":
    rospy.init_node("learnObject_calculateLearnPoses")
    PoseCalculator()
    rospy.spin()

