#!/usr/bin/env python
import yaml
import rospy
import os
import errno
from geometry_msgs.msg import PoseStamped
import actionlib
from learn_object.msg import cameraAction, cameraGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from learn_object.msg import saveFrame
from learn_object.srv import saveFrame,saveFrameResponse


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

def movebase_client(target, index):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    while (not client.wait_for_server(rospy.Duration(4))) and ( not rospy.is_shutdown() ):
        rospy.loginfo("%d: Waiting for Move Base Server to come up" %index)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = target.header.frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = target.pose.position.x
    goal.target_pose.pose.position.y = target.pose.position.y
    goal.target_pose.pose.position.z = target.pose.position.z
    goal.target_pose.pose.orientation.x = target.pose.orientation.x
    goal.target_pose.pose.orientation.y = target.pose.orientation.y
    goal.target_pose.pose.orientation.z = target.pose.orientation.z
    goal.target_pose.pose.orientation.w = target.pose.orientation.w

    client.send_goal(goal)
    finished_within_time = client.wait_for_result(rospy.Duration(60))
    if not finished_within_time:
        rospy.logerr("Achieving goal was not successful")
        # rospy.signal_shutdown("Action server not available!")
    else:
        print(client.get_result())    
        return client.get_result()



if __name__=="__main__":
    # try:
    rospy.init_node("learnObject_takePhotos") 

    # Get Parameter
    filename_poses    = rospy.get_param('~filename_poses', '')
    path_object       = rospy.get_param('~path_object', '')
    object_name       = rospy.get_param('~object_name', 'object')
    poses=dict()
    
    # Load Poses from .yaml
    if os.path.isfile(filename_poses):
        with open(filename_poses, 'r') as infile:
            raw=yaml.safe_load(infile)
            if raw:
                poses=dictToPose(raw)  
        # print("Loaded pose: ")
        # print(poses) 
    else:
        rospy.logerr("Could not load %s" %filename_poses )    
    
    # Wait for Camera Server to come up
    cam_client = actionlib.SimpleActionClient('camera_stream', cameraAction)
    while( not cam_client.wait_for_server(rospy.Duration(4)) ):
        rospy.loginfo("Waiting for Camera Server to come up")
    
    start_cam = cameraGoal()
    start_cam.stream = True
    cam_client.send_goal(start_cam)
    
    # cam_pub = rospy.Publisher('Camera/save_Frame', saveFrame)
    # msg = saveFrame()
    # msg.path = path_object

    # Loop
    # Navigate to Pose, when SUCCEDED take a pohto and save it
    for elem in poses:
        # Get PoseStamped
        pose=poses[elem]   
        # Nav to pose_msg
        result = movebase_client(pose, elem)
        if result:
            rospy.loginfo("Reached pose %d" %elem)
            # Take Photo
            # msg.filename = object_name + str(elem) + '.jpg'
            # cam_pub.publish(msg)
            rospy.wait_for_service('camera_saveFrame')
            try:
                save_srv = rospy.ServiceProxy('camera_saveFrame', saveFrame)
                saved = save_srv(path_object, object_name + str(elem) + '.jpg')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            rospy.sleep(2.0)
    
    print("Done Taking Photos")

    cam_client.cancel_goal()   
    cam_client.wait_for_result(rospy.Duration(30))
    # if not canceld_within_time:
    #     rospy.logerr("was canceld")
    # else:
    #     print(cam_client.get_result())
    #     rospy.signal_shutdown("Action server not available!")    

    # except Exception as e:
    #     print(e)
    #     pass 
