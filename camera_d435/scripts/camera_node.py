#!/usr/bin/env python
import rospy
import os
import errno
# import actionlib
# from learn_object.msg import cameraAction, cameraResult

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
        self.__profile = self.__pipeline.get_active_profile()
        self.__saveFrame_srv=rospy.Service("camera_saveFrame",saveFrame,self.__saveFrameSrv__)

    
    def __del__(self):        
        self.__pipeline.stop()
        cv2.destroyAllWindows()

    def __printParams__(self):
        depth_profile = rs.video_stream_profile(self.__profile.get_stream(rs.stream.depth))
        color_profile = rs.video_stream_profile(self.__profile.get_stream(rs.stream.color))
        depth_intrinsics = depth_profile.get_intrinsics()
        color_intrinsics = color_profile.get_intrinsics()
        cd = color_intrinsics.coeffs
        dd = depth_intrinsics.coeffs
        # print("Kamera Parameter")
        # print(depth_profile)
        # print(color_profile)        
        rospy.loginfo("Sensor internal camera parameters for color camera: ")
        # print(color_intrinsics)
        rospy.loginfo("  Principal point: \t px = %f \t py = %f", color_intrinsics.ppx, color_intrinsics.ppy)
        rospy.loginfo("  Focal length:    \t fx = %f \t fy = %f", color_intrinsics.fx, color_intrinsics.fy)
        rospy.loginfo("  Distortion coefficients: %.1f %.1f %.1f %.1f", cd[0], cd[1], cd[2], cd[3])
        rospy.loginfo("Sensor internal camera parameters for depth camera: ")
        # print(depth_intrinsics)
        rospy.loginfo("  Principal point: \t px = %f \t py = %f", depth_intrinsics.ppx, depth_intrinsics.ppy)
        rospy.loginfo("  Focal length:    \t fx = %f \t fy = %f", depth_intrinsics.fx, depth_intrinsics.fy)
        rospy.loginfo("  Distortion coefficients: %.1f %.1f %.1f %.1f", dd[0], dd[1], dd[2], dd[3])
                

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

    def __stream__(self):       
        try: 
            self.__frames = self.__pipeline.wait_for_frames()
            self.__depth_frame = self.__frames.get_depth_frame()
            self.__color_frame = self.__frames.get_color_frame()
            if  self.__color_frame:   
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
        except Exception as e:
            print(e)
            pass

if __name__=="__main__":
    rospy.init_node("learnObject_cameraServer")
    cam = Camera()
    rospy.loginfo("Start Streaming")
    cam.__printParams__()
    while not rospy.is_shutdown():
        cam.__stream__()

   
    
