#!/usr/bin/env python
from learn_object.OnePoseTeacher import PoseFileHandler
import rospy
if __name__=="__main__":
    rospy.init_node("learnObject_TeachOnePose")
    handler=PoseFileHandler()
    rospy.spin()
