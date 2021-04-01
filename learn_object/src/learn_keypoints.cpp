
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>         // in createTrainImages to create model folder

#include <unistd.h>
#include <iostream>
#include <std_msgs/String.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/vision/vpKeyPoint.h>        // vpKeyPoint
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

#include <sstream>
#include <string>
#include <fstream>

template<class T>
std::string toString(const T &value) {
    std::ostringstream os;
    os << value;
    return os.str();
}


class Detection
{
protected:
    ros::NodeHandle n_;    
    std::string path_; 
    std::string pathModel_;  
    std::string feature_;
    std::string learningExtension_;
    int maxTrainImageNumber_;
    bool binMode_;
    bool saveKeypointsGlobal_;
    std::vector<int> numberKeypoints_;

    // Camera Settings
    vpCameraParameters cam_color_; 
    vpImage<vpRGBa> Icolor_; 

    std::string detectorName_;
    std::string extractorName_;
    std::string matcherName_;  
    std::string configurationFile_; 
    
public:
    std::string objectName_;     

    Detection(std::string name)
    {        
        ros::NodeHandle nh("~");
        nh.param<std::string>("object_name", objectName_, "Teabox" );                               // name of object to learn
        nh.param<std::string>("feature", feature_, "ORB");                                           // select feature for the keypoints (e.g. "SURF", "SIFT")
        nh.param<std::string>("path", pathModel_, "");                                                   // path to mir_vision (where model/ and config/ are saved)
        path_ = pathModel_ + objectName_ + "/";
        nh.param<bool>("saveKeypointsGlobal", saveKeypointsGlobal_, false);                                                  // If true: learningData is saved as .bin, otherwise it is saved as .xml
        nh.param<bool>("binMode", binMode_, true);                                                  // If true: learningData is saved as .bin, otherwise it is saved as .xml
        learningExtension_ = binMode_ ? ".bin" : ".xml";              
        nh.param<int>("maxTrainImageNumber", maxTrainImageNumber_, 3);                               // number of image(s) to save
    }

    ~Detection(void) {}      


    void setDetectionSettings(std::string name, vpKeyPoint& keypoint)
    {
        if (name == "ORB"){
            detectorName_ = name;    // "FAST"
            extractorName_ = name;
            matcherName_ = "BruteForce-Hamming";
            configurationFile_ ="detection-config.xml";
        }
        if (name == "SIFT" || "SURF"){
            detectorName_ = name;
            extractorName_ = name;
            matcherName_ = "FlannBased"; // "BruteForce"
            configurationFile_ ="detection-config-" + name + ".xml";
        }  
        keypoint.setDetector(detectorName_);
        keypoint.setExtractor(extractorName_);
        keypoint.setMatcher(matcherName_);
        keypoint.setFilterMatchingType(vpKeyPoint::ratioDistanceThreshold);               
    }

    void learnCube(const vpImage<vpRGBa> &I, vpMbGenericTracker &tracker, vpKeyPoint &keypoint_learning, int id)
    {
        // *** Detect keypoints on frame I
        std::vector<cv::KeyPoint> trainKeyPoints;
        double elapsedTime; 
        keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
        
        // *** Get Polygons and their vertex/corners (rois)
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
        polygons = pair.first;
        roisPt = pair.second;
        // *** Calculate HomogenousMatrix and the 3D coordinates of the keypoints
        std::vector<cv::Point3f> points3f;
        vpHomogeneousMatrix cMo;
        tracker.getPose(cMo);
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam_color_, trainKeyPoints, polygons, roisPt, points3f);
        // *** Save the Keypoints and their 3D coordingates in the keypoint and a file (per image)
        keypoint_learning.buildReference(I, trainKeyPoints, points3f, true, id);    // (..., points3f, bool append, calss id)  
        // keypoint_learning.saveLearningData(objectNumberPath + "_learning_data.bin", true, false);
        numberKeypoints_.push_back(trainKeyPoints.size());

        // *** Display the learned Keypoints
        vpDisplay::display(Icolor_);
        for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
            vpDisplay::displayCross(Icolor_, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
        }                tracker.track(Icolor_);
        tracker.getPose(cMo);
        tracker.display(Icolor_, cMo, cam_color_, vpColor::red);
        vpDisplay::displayText(Icolor_, 10, 10, "Learning step: keypoints are detected on visible teabox faces", vpColor::red);
    }
    
    void saveNumberKeypoints(std::string filename, bool saveGlobal = false){
        std::ofstream file;        
        file.open((path_ + filename).c_str(), std::ifstream::out); 
        if (file.is_open())
        {
           file << objectName_ << std::endl; 
           file << "image number; number of keypoints" << std::endl;
           for (int i = 1; i <= maxTrainImageNumber_; i++)
           {
               file << i << "; " << numberKeypoints_[i-1] << std::endl;
           }
           file.close();
        } else ROS_INFO("Unable to open file %s", (path_ + filename).c_str() ); 

        if (saveGlobal) {
            std::ofstream f;
            f.open((pathModel_ + filename).c_str(), std::ifstream::app); 
            if (f.is_open())
            {
                f << objectName_ << "; " ;          
                for (int i = 1; i <= maxTrainImageNumber_; i++)
                {
                    f << numberKeypoints_[i-1] << "; ";
                }
                f  <<  std::endl; 
                f.close();
            } else ROS_INFO("Unable to open file %s", (pathModel_ + filename).c_str() ); 

        }
 
    } 

    void createLearnImages(){
        // Camera Parameter
        Icolor_.resize(480, 640);
        double u0 = Icolor_.getWidth()  / 2.;   // 319.999634 principal point
        double v0 = Icolor_.getHeight() / 2.;   // 237.854675
        double fx = 615.718018;                 // ratio between focal length and size of a pixel. 
        double fy = 616.188049;
        cam_color_.initPersProjWithoutDistortion(fx, fy, u0, v0);
        // vpRealSense2 realsense;
        // cam_color_ = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion); //perspectiveProjWithDistortion        
        cam_color_.printParameters();
        ROS_INFO("Sensor internal camera parameters for color camera: ");
        ROS_INFO("  px = %f \t py = %f", cam_color_.get_px(), cam_color_.get_py());
        ROS_INFO("  u0 = %f \t v0 = %f", cam_color_.get_u0(), cam_color_.get_v0());
        ROS_INFO("  kud = %f \t kdu = %f", cam_color_.get_kud(), cam_color_.get_kdu()); 
        
        vpDisplayOpenCV d_c;
        d_c.init(Icolor_,100, 50, "Color Stream"); 
        
        std::string objectPath = path_ + objectName_;
        std::string objectNumberPath;
        std::string ext = ".jpg";

        // *** Tracker and Keypoint Settings
        vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
        vpKeyPoint keypoint_learning;                  
        bool usexml = false;
        if (vpIoTools::checkFilename(objectPath + ".xml")) {
            tracker.loadConfigFile(objectPath + ".xml");
            tracker.getCameraParameters(cam_color_);
            usexml = true;
        }
        if (!usexml) {
            vpMe me;
            me.setMaskSize(5);
            me.setMaskNumber(180);
            me.setRange(8);
            me.setThreshold(10000);
            me.setMu1(0.5);
            me.setMu2(0.5);
            me.setSampleStep(4);
            me.setNbTotalSample(300);
            tracker.setMovingEdge(me);
            // cam.initPersProjWithoutDistortion(839, 839, 325, 243);
            tracker.setCameraParameters(cam_color_);
            tracker.setAngleAppear(vpMath::rad(70));
            tracker.setAngleDisappear(vpMath::rad(80));
            tracker.setNearClippingDistance(0.1);
            tracker.setFarClippingDistance(100.0);
            tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
        }
        tracker.setOgreVisibilityTest(false);
        tracker.setDisplayFeatures(true);
        if (vpIoTools::checkFilename(objectPath + ".cao"))
            tracker.loadModel(objectPath + ".cao");
        else if (vpIoTools::checkFilename(objectPath + ".wrl"))
            tracker.loadModel(objectPath + ".wrl"); 

        // *** Keypoint settings
        setDetectionSettings(feature_, keypoint_learning);
        if (usexml) { keypoint_learning.loadConfigFile(path_ + configurationFile_); } 
               

        for (int i = 1; i <= maxTrainImageNumber_; i++) 
        {
            ROS_INFO("Image Number: %i", i);  
            objectNumberPath = objectPath + std::to_string(i);  
            vpImageIo::read(Icolor_, objectNumberPath + "_color" + ext); 
 
            tracker.initClick(Icolor_, objectNumberPath + ".init", true);
            learnCube(Icolor_, tracker, keypoint_learning, i);               
            
        
            if (i < maxTrainImageNumber_) {
                vpDisplay::displayText(Icolor_, 30, 10, toString(i) + "/" + toString(maxTrainImageNumber_) + "done. Click to learn next pose of the object.", vpColor::red);
            } else {
                 vpDisplay::displayText(Icolor_, 30, 10, toString(i) + "/" + toString(maxTrainImageNumber_) + "done. Click to finish learning.", vpColor::red);
            } 
            vpDisplay::flush(Icolor_);
            vpDisplay::getClick(Icolor_, true);        
        }       
        keypoint_learning.saveLearningData(objectNumberPath + "_learning_data" + learningExtension_, binMode_, false);   // (filename, binaryMode, saveTrainImages)
        saveNumberKeypoints("NumberKeypoints.txt", saveKeypointsGlobal_);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "learnObject_learnKeypoints"); 
    Detection detection("LearnObject"); 

    ROS_INFO("Detection node to learn object %s started", detection.objectName_.c_str());
     
    detection.createLearnImages();

    ros::shutdown();
    return 0;
}