#!/usr/bin/env python
import rospy
import os
import errno
import actionlib
from learn_object.msg import cameraAction, cameraResult

# import actionlib_tutorials.msg
import pyrealsense2 as rs
import numpy as np
import cv2
# from learn_object.msg import saveFrame
from learn_object.srv import saveFrame,saveFrameResponse

class Camera:   
    def __init__(self):
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)        
        self.__pipeline.start(self.__config)
        # self.__frameSub=rospy.Subscriber("Camera/save_Frame",saveFrame,self.__saveFrameSub__)
        self.__saveFrame_srv=rospy.Service("camera_saveFrame",saveFrame,self.__saveFrameSrv__)

        self._as = actionlib.SimpleActionServer("camera_stream", cameraAction, execute_cb=self.__execute_cb, auto_start = False)
        self._as.start()
        self.__result = cameraResult()
    
    
    def __del__(self):        
        self.__pipeline.stop()
        cv2.destroyAllWindows()

    def __saveFrameSrv__(self, req):
        rospy.loginfo("Saving frame")
        if not os.path.exists(os.path.dirname(req.path)):
            try:
                os.makedirs(os.path.dirname(req.path))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
                return saveFrameResponse(False)
        cv2.imwrite(req.path + req.filename, self.__color_image)
        return saveFrameResponse(True)
    
    # def __saveFrameSub__(self, msg):
    #     rospy.loginfo("Saving frame")
    #     if not os.path.exists(os.path.dirname(msg.path)):
    #         try:
    #             os.makedirs(os.path.dirname(msg.path))
    #         except OSError as exc: # Guard against race condition
    #             if exc.errno != errno.EEXIST:
    #                 raise
    #     cv2.imwrite(msg.path + msg.filename, self.__color_image)

    def __execute_cb(self, goal):
        rospy.loginfo("Start Streaming")
        self.__result.success = True
        
        while (not self._as.is_preempt_requested()) and (not rospy.is_shutdown()) :
            self.__frames = self.__pipeline.wait_for_frames()
            self.__depth_frame = self.__frames.get_depth_frame()
            self.__color_frame = self.__frames.get_color_frame()
            if not self.__depth_frame or not self.__color_frame: 
                continue
            # Convert images to numpy arrays
            # depth_image = np.asanyarray(self.__depth_frame.get_data())
            self.__color_image = np.asanyarray(self.__color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # depth_colormap_dim = depth_colormap.shape
            # color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            # if depth_colormap_dim != color_colormap_dim:
            #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            #     images = np.hstack((resized_color_image, depth_colormap))
            # else:
            #     images = np.hstack((color_image, depth_colormap))

            # Show image(s)
            images = self.__color_image
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                self.__result.success = False                
                break

        if self._as.is_preempt_requested():
            self._as.set_preempted()
            self.__result.success = False
        else:
            self._as.set_succeeded(self.__result.success)


if __name__=="__main__":
    rospy.init_node("learnObject_cameraServer")
    cam_server = Camera()
    rospy.spin()
   
    
