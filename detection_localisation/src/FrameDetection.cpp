// #define VISP_HAVE_REALSENSE2 true
// #define VISP_HAVE_OPENCV true
// #if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OPENCV)

#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"             // Subscribe to Odom
#include <boost/filesystem.hpp>         // in createTrainImages to create model folder

#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>                      // for saving files 
#include <math.h>                       // for pi

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



#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


class CamDetection
{
protected:

    ros::NodeHandle n_;
    ros::Subscriber sub_;
   
    int STATUS;
    int STATUS_CHECK_DATA = 0;
    int STATUS_INIT = 1;
    int STATUS_IDLE = 2;
    int STATUS_TEST = 3;
    int STATUS_DETECT_IN_FILE = 5;     
    int STATUS_DETECT_WITH_CAM = 10;
    int STATUS_END_DETECTION = 500;
    int STATUS_EXIT = 999;  

    int MACTHES_THRESHOLD = 33;
    
    std::string targetName_;
    std::string objectName_;
    std::string pathModel_;      
    std::string pathTargetFolder_; 
    std::string modelFile_;
    std::string learningData_;
    std::string detectorName_;
    std::string extractorName_;
    std::string matcherName_;  
    std::string configurationFile_;

    // Camera Settings
    vpRealSense2 realsense_;
    vpDisplayOpenCV d_c, d_d, d_g;
    vpCameraParameters camColor_, camDepth_; 
    vpImage<vpRGBa> imgColor_; 
    vpImage<unsigned char> I_depth_, I_gray_;
    vpImage<uint16_t> I_depth_raw_; 
    vpHomogeneousMatrix cMo_;
    std::vector<vpHomogeneousMatrix> cMoVec_; 

    std::vector<double> matchesVec_, matchesStat_;
    std::vector<double> pxVec_, pyVec_, pzVec_, rotxVec_, rotyVec_, rotzVec_;
    std::vector<double> pxStat_, pyStat_, pzStat_, rotxStat_, rotyStat_, rotzStat_;

    std::vector<double> times_vec;    
    bool useCam_, verbose_, useColor_;
    double start, looptime, fps_measured;  
    int pose_, nr_, nrThreshold_, nrPoses_, matches_;    
    
public:
    CamDetection(std::string name)
    {        
        ros::NodeHandle nh("~");
        nh.param<std::string>("path_package", pathModel_, "src/object_localisation_d435/learn_object/model/");
        nh.param<std::string>("objectName", objectName_, "Goesser");
        nh.param<std::string>("targetName", targetName_, "Goesser_27_56_");
        nh.param<std::string>("learning_data", learningData_, "");
        nh.param<bool>("useCam", useCam_, false);
        nh.param<bool>("useColorImg", useColor_, false);
        nh.param<bool>("verbose", verbose_, true);
        nh.param<int>("numberOfPoses", nrPoses_, 12);
        nh.param<int>("numberImagesPerPose", nrThreshold_, 20);
        cMoVec_.assign(nrThreshold_, cMo_);
        matchesVec_.assign(nrThreshold_, matches_);
        pathTargetFolder_ = pathModel_ + targetName_ + "/";
        pose_ = 1;
        nr_ = 0;
    }

    ~CamDetection(void) {}

    bool startCamera(int width, int height, int fps)
    {   
        // *** Configuration
        rs2::config config;        
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        // *** Try to open Camera
        try { realsense_.open(config);}
        catch (const vpException &e) {
            ROS_ERROR("Catch an exception: %s", e.what());
            ROS_INFO("Check if the Realsense camera is connected...");
            return false;}
        // *** Get and print Parameters
        camColor_ = realsense_.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion); //perspectiveProjWithDistortion
        camDepth_ = realsense_.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);
        if (verbose_){
            ROS_INFO("Sensor internal camera parameters for color camera: ");
            ROS_INFO("  px = %f \t py = %f", camColor_.get_px(), camColor_.get_py());
            ROS_INFO("  u0 = %f \t v0 = %f", camColor_.get_u0(), camColor_.get_v0());
            ROS_INFO("  kud = %f \t kdu = %f", camColor_.get_kud(), camColor_.get_kdu());
            ROS_INFO("Sensor internal camera parameters for depth camera: " );
            ROS_INFO("  px = %f \t py = %f", camDepth_.get_px(), camDepth_.get_py());
            ROS_INFO("  u0 = %f \t v0 = %f", camDepth_.get_u0(), camDepth_.get_v0());
            ROS_INFO("  kud = %f \t kdu = %f", camDepth_.get_kud(), camDepth_.get_kdu());
        }        
        // *** Resize images
        imgColor_.resize(height, width);
        I_gray_.resize(height, width);
        I_depth_.resize(height, width);
        I_depth_raw_.resize(height, width);
        return true;       
    }

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
    
    void getFrameCam(){
        looptime = vpTime::measureTimeMs() - start;
        times_vec.push_back(looptime);
        fps_measured = 1/(looptime/1000);
        start = vpTime::measureTimeMs();

        realsense_.acquire((unsigned char *) imgColor_.bitmap, (unsigned char *) I_depth_raw_.bitmap, NULL, NULL);
        if (useColor_) {
            vpDisplay::display(imgColor_);
            vpDisplay::displayText(imgColor_, 30, 10, "FPS measured: " + std::to_string(fps_measured) , vpColor::red);
        } else {
            vpImageConvert::convert(imgColor_, I_gray_);
            vpDisplay::display(I_gray_);
            vpDisplay::displayText(I_gray_, 30, 10, "FPS measured: " + std::to_string(fps_measured) , vpColor::red);
        }      
    }

    void getFrameFile(){
        
        nr_++;
        if (nr_ > nrThreshold_){
            nr_ = 1;            
            pose_++;
        }
        vpImageIo::read(imgColor_, pathTargetFolder_ + "linear/" + targetName_ + "Pose"+ std::to_string(pose_) +"_nr"+ std::to_string(nr_)+ ".jpg");
        vpDisplay::display(imgColor_);
    }


    bool validPose(){
        bool valid_status = false;
        // vpTranslationVector trans;
        // cMo_.extract(trans);
        float transY = cMo_[1][3];
        float transZ = cMo_[2][3];
        float hk = 0.55;
        float alpha = 16*(M_PI/180);
        float tolerance = 0.1;        
        float d = sqrt(pow(transY,2) + pow(transZ,2));
        float resultH = d*sin(alpha);

        // ROS_INFO("Y: %f; Z: %f; result hk: %f", transY, transZ, d);
        if ( resultH > hk+tolerance || resultH < hk-tolerance){
            ROS_INFO("Valid H");
            valid_status  =true;
        }
        return valid_status;
    }

    void executeCB()
    {
        double start = vpTime::measureTimeSecond();
        STATUS = STATUS_CHECK_DATA;
        // STATUS = STATUS_TEST;
        bool proceed = true;        
        int frame = 0;
        vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
        vpKeyPoint keypoint;

        while (proceed && ros::ok()) {
        
            switch (STATUS)
            {
                case 0:     // STATUS_CHECK_DATA
                {
                    ROS_INFO("STATUS: CHECK DATA AND REQUEST");
                    STATUS = STATUS_INIT; 
                    
                    // *** Check if name of detection goal is given
                    if (targetName_ == "" ) {
                        ROS_ERROR("No targetname given." );
                        STATUS = STATUS_EXIT; 
                    }              
                    
                    // Check files for object model                    
                    if (vpIoTools::checkFilename(pathTargetFolder_ + objectName_ + ".cao")) {
                        modelFile_ = pathTargetFolder_ + objectName_ + ".cao";
                    } else if (vpIoTools::checkFilename(pathTargetFolder_ + objectName_ + ".wrl")) {
                        modelFile_ = pathTargetFolder_ + objectName_ + ".wrl";
                    } else {
                        ROS_ERROR("Can not open target modelfile(.cao or .wrl): %s", (pathTargetFolder_ + objectName_).c_str() );
                        STATUS = STATUS_EXIT;                        
                    }

                    // *** Check file for learning data
                    if (!vpIoTools::checkFilename(pathTargetFolder_ + learningData_)) {
                        ROS_ERROR("Can not open learning data: %s", (pathTargetFolder_+learningData_).c_str());            
                        STATUS = STATUS_EXIT;
                    }              

                    // *** Check Camera start
                    int width = 640, height = 480, fps = 30;
                    if (useCam_)
                    {                                             
                        if (!startCamera(width, height, fps)) {
                            ROS_ERROR("Can not open Camera");
                            STATUS = STATUS_EXIT;
                        }

                    } else {
                        imgColor_.resize(height, width);
                        double u0 = imgColor_.getWidth()  / 2.;   // 319.999634 principal point
                        double v0 = imgColor_.getHeight() / 2.;   // 237.854675
                        double fx = 615.718018;                 // ratio between focal length and size of a pixel. 
                        double fy = 616.188049;
                        camColor_.initPersProjWithoutDistortion(fx, fy, u0, v0);
                    }
                    
                    ROS_INFO("Valid detection request and input data.");  
                    break;
                }
                case 1:     // STATUS_INIT
                {
                    ROS_INFO("STATUS: INIT");
                    // *** Display settings                    
                    unsigned int _posx = 100, _posy = 50; 
                    if (useColor_) {
                        d_c.init(imgColor_, _posx, _posy, "Color stream");
                    } else {
                        d_g.init(I_gray_, _posx, _posy, "Gray stream");
                    }                            
                    // d_d.init(       I_depth, _posx + imgColor_.getWidth()+10,  _posy,                          "Depth stream");

                    // *** Tracker settings
                    tracker.setOgreVisibilityTest(false);
                    tracker.setDisplayFeatures(true);
                    tracker.loadModel(modelFile_);
                    if (vpIoTools::checkFilename(pathTargetFolder_ + objectName_ + ".xml")) {
                        tracker.loadConfigFile(pathTargetFolder_ + objectName_ + ".xml");
                        tracker.getCameraParameters(camColor_);
                    } else {
                        vpMe me;
                        me.setMaskSize(5);
                        me.setMaskNumber(180);
                        me.setRange(8);
                        me.setThreshold(10000);
                        me.setMu1(0.5);
                        me.setMu2(0.5);
                        me.setSampleStep(4);
                        me.setNbTotalSample(250);
                        tracker.setMovingEdge(me);
                        // cam.initPersProjWithoutDistortion(839, 839, 325, 243);
                        tracker.setCameraParameters(camColor_);
                        tracker.setAngleAppear(vpMath::rad(98));
                        tracker.setAngleDisappear(vpMath::rad(98));
                        tracker.setNearClippingDistance(0.1);
                        tracker.setFarClippingDistance(100.0);
                        tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
                    }
                    
                    // *** Keypoint Settings (detector, extractor, matcher, learning data)                   
                    setDetectionSettings("SURF", keypoint);
                    bool binMode = ( learningData_.rfind(".bin") != std::string::npos );
                    keypoint.loadLearningData(pathTargetFolder_ + learningData_, binMode); 
                    if (vpIoTools::checkFilename(pathTargetFolder_ + configurationFile_)){                    
                        keypoint.loadConfigFile(pathTargetFolder_ + configurationFile_);
                    } else {            
                        keypoint.setMatchingRatioThreshold(0.8);
                        keypoint.setUseRansacVVS(true);
                        keypoint.setUseRansacConsensusPercentage(true);
                        keypoint.setRansacConsensusPercentage(20.0);
                        keypoint.setRansacIteration(200);
                        keypoint.setRansacThreshold(0.005);
                    }
                    STATUS = useCam_ ?  STATUS_DETECT_WITH_CAM : STATUS_DETECT_IN_FILE;
                    ROS_INFO("INIT done");
                    start = vpTime::measureTimeMs();
                    break;
                }

                case 2:     // STATUS_IDLE
                {
                    break;
                }

                case 3:     // STATUS_TEST
                {  
                    ROS_INFO("STATUS TEST");                   
                    STATUS = STATUS_IDLE;
                    break;
                }

                case 5:     // STATUS_DETECT_IN_FILE
                {                                        
                    double error, elapsedTime;
                    unsigned int matches;
                    vpHomogeneousMatrix H;

                    getFrameFile();

                    keypoint.matchPoint(imgColor_, camColor_, H, error, elapsedTime);
                    matches = keypoint.getMatchedPointNumber();                                           
                    tracker.setPose(imgColor_, H);

                    // ROS_INFO("Pose: %i; nr: %i", pose_, nr_);
                    if (nr_ == 1) {
                        cMoVec_.clear();
                        pxVec_.clear();
                        pyVec_.clear();
                        pzVec_.clear();
                        rotxVec_.clear();
                        rotyVec_.clear();
                        rotzVec_.clear();
                        matchesVec_.clear();
                        matchesStat_.clear();
                    }

                    cMoVec_.push_back(H);
                    matchesVec_.push_back(matches);

                    if (nr_ >= nrThreshold_){
                        computeStatistics(cMoVec_);
                        matchesStat_ = getStatistic(matchesVec_);
                        // saveData();
                    }
                    
                    tracker.display(imgColor_, H, camColor_, vpColor::red, 2);                   
                    vpDisplay::displayFrame(imgColor_, H, camColor_, 0.025, vpColor::none, 3);
                    vpDisplay::flush(imgColor_);                    
                    if (pose_ == nrPoses_ && nr_>= nrThreshold_) {STATUS = STATUS_EXIT;}
                    break; 
                }    
                case 10:    // STATUS_DETECT_WITH_CAM                                    
                {    
                    getFrameCam();
                    double error, elapsedTime;
                    if (useColor_){
                        keypoint.matchPoint(imgColor_, camColor_, cMo_, error, elapsedTime);
                    } else {
                        keypoint.matchPoint(I_gray_, camColor_, cMo_, error, elapsedTime);
                    }
                    
                    unsigned int matches = keypoint.getMatchedPointNumber();

                    float distance = sqrt(pow(cMo_[1][3],2) + pow(cMo_[2][3],2));
                    ROS_INFO("Matches: %i; Distanz Cam-Ob: %f", matches,distance);                    
                    if (matches > MACTHES_THRESHOLD ) //&& validPose())
                    {
                        if (useColor_){
                            tracker.setPose(imgColor_, cMo_);   
                            tracker.display(imgColor_, cMo_, camColor_, vpColor::red, 2);                   
                            vpDisplay::displayFrame(imgColor_, cMo_, camColor_, 0.025, vpColor::none, 3);
                        } else {
                            tracker.setPose(I_gray_, cMo_);   
                            tracker.display(I_gray_, cMo_, camColor_, vpColor::red, 2);                   
                            vpDisplay::displayFrame(I_gray_, cMo_, camColor_, 0.025, vpColor::none, 3);
                        }                        
                    } 
                    if (useColor_) {
                        vpDisplay::flush(imgColor_);
                    } else {
                        vpDisplay::flush(I_gray_);
                    }
                    break;
                }                   
                case 500:   // STATUS_END_DETECTION
                {
                    ROS_INFO("STATUS: EXECUTION FINISHED!");
                    // if (verbose_) {
                    //     double detectionTime = vpTime::measureTimeSecond() - start;                        
                    //     ROS_INFO("Processing time: %.2f min", detectionTime/60);
                    // } 
                    // // saveData();
                    //     result_.angles = getAngles(cMo_);
                    //     server_.setSucceeded(result_);
                    //     ROS_INFO("SEARCH SUCCEEDED: FOUND %s!", objectName_.c_str());
                    //   break;
                }
                case 999:   // STATUS_EXIT LOOP
                {
                    ROS_INFO("STATUS: EXIT CALLBACK");
                    proceed = false;  
                    break;  
                }     
            }
        }
    }

    void saveData()
    {
        std::string filename = pathTargetFolder_ + "/linear/DetectionDataLog.txt";
        std::ofstream f;

        if (pose_ == 1) {
            f.open(filename.c_str(), std::ifstream::out);
            if (f.is_open())
            {
                f << "Linear Detection Measurement - " << targetName_ << std::endl;
                f << "Pose p; Framenumber n; number of detected matches; targetpose px[m]; py[m]; pz[m]; rotx[°]; roty[°]; rotz[°]" << std::endl;
                f << "Pose p; calculate Median" << std::endl;
                f << "Pose p; calculate Mittelwert" << std::endl;
                f << "Pose p; calculate Spannweite" << std::endl;
                f << "Pose p; calculate Stdev" << std::endl;
            }
            f.close();
        }


        f.open(filename.c_str(), std::ifstream::app);
        if (f.is_open())
        {
            for (int n = 1; n <= nrThreshold_; n++)
            {
                f << pose_<< "; " << n << "; " << matchesVec_[n-1] << "; " << pxVec_[n-1] << "; " << pyVec_[n-1] << "; " << pzVec_[n-1] << "; ";
                    f << rotxVec_[n-1] << "; " << rotyVec_[n-1] << "; " << rotzVec_[n-1] << std::endl;
            }
            f << pose_ << "; Median; "  << matchesStat_[0] << "; " << pxStat_[0] << "; " << pyStat_[0] << "; ";
                f << pzStat_[0] << "; " << rotxStat_[0] << "; " << rotyStat_[0] << "; " << rotzStat_[0] << std::endl;
            f << pose_ << "; Mean; "  << matchesStat_[1] << "; " << pxStat_[1] << "; " << pyStat_[1] << "; ";
                f << pzStat_[1] << "; " << rotxStat_[1] << "; " << rotyStat_[1] << "; " << rotzStat_[1] << std::endl;
            f << pose_ << "; Range; "  << matchesStat_[2] << "; " << pxStat_[2] << "; " << pyStat_[2] << "; ";
                f << pzStat_[2] << "; " << rotxStat_[2] << "; " << rotyStat_[2] << "; " << rotzStat_[2] << std::endl;               
            f << pose_ << "; Stdev; "  << matchesStat_[3] << "; " << pxStat_[3] << "; " << pyStat_[3] << "; ";
                f << pzStat_[3] << "; " << rotxStat_[3] << "; " << rotyStat_[3] << "; " << rotzStat_[3] << std::endl;
           f.close();
           ROS_INFO("Saved Pose %i", pose_);
        } else ROS_INFO("Unable to open file %s", filename.c_str() );   

        std::string filename2 = pathModel_ + "DetectionDataLog.txt";
        std::ofstream f2; 
        f2.open(filename2.c_str(), std::ifstream::app);
        if (f2.is_open())
        {
            f2 << targetName_ << "; " << pose_ << "; Median; "  << matchesStat_[0] << "; " << pxStat_[0] << "; " << pyStat_[0] << "; ";
                f2 << pzStat_[0] << "; " << rotxStat_[0] << "; " << rotyStat_[0] << "; " << rotzStat_[0] << std::endl;
            f2 << targetName_ << "; " << pose_ << "; Mean; "  << matchesStat_[1] << "; " << pxStat_[1] << "; " << pyStat_[1] << "; ";
                f2 << pzStat_[1] << "; " << rotxStat_[1] << "; " << rotyStat_[1] << "; " << rotzStat_[1] << std::endl;
            f2 << targetName_ << "; " << pose_ << "; Range; "  << matchesStat_[2] << "; " << pxStat_[2] << "; " << pyStat_[2] << "; ";
                f2 << pzStat_[2] << "; " << rotxStat_[2] << "; " << rotyStat_[2] << "; " << rotzStat_[2] << std::endl;               
            f2<< targetName_ << "; " << pose_ << "; Stdev; "  << matchesStat_[3] << "; " << pxStat_[3] << "; " << pyStat_[3] << "; ";
                f2 << pzStat_[3] << "; " << rotxStat_[3] << "; " << rotyStat_[3] << "; " << rotzStat_[3] << std::endl;
           f2.close();
           ROS_INFO("Saved Pose %i global", pose_);
        } else ROS_INFO("Unable to open file %s", filename2.c_str() );       

    }
    
    double getMedian(std::vector<double> vec)
    {
        double median;
        std::nth_element(vec.begin(), vec.begin()+vec.size()/2, vec.end());
        median = vec[vec.size()/2];
        return median;
    }

    double getRange(std::vector<double> vec){
        double range, maxVal, minVal;
        maxVal = *std::max_element(vec.begin(), vec.end());
        minVal = *std::min_element(vec.begin(), vec.end());
        range = maxVal - minVal;
        return range;
    }

    std::vector<double> getStatistic(std::vector<double> vec){
        std::vector<double> statVec;
        double median, mean, range, stdev;

        median  = getMedian(vec);
        mean    = vpMath::getMean(vec);
        range   = getRange(vec);
        stdev   = vpMath::getStdev(vec);

        statVec.push_back(median);
        statVec.push_back(mean);
        statVec.push_back(range);
        statVec.push_back(stdev);       

        return statVec;
    }

    void computeStatistics(std::vector<vpHomogeneousMatrix>& M)
    {       

        // ROS_INFO("cMoVec size is %zu", cMoVec_.size());
        for (int i = 0; i < M.size(); i++)
        {
            pxVec_.push_back(M[i][0][3]);
            pyVec_.push_back(M[i][1][3]);
            pzVec_.push_back(M[i][2][3]);

            Eigen::Vector3d euler_angles = getRPY(M[i], false, true);
            rotzVec_.push_back(euler_angles[0]);    // gier = yaw
            rotyVec_.push_back(euler_angles[1]);    // nick = pitch
            rotxVec_.push_back(euler_angles[2]);    // roll

            // ROS_INFO("px: %f; py: %f; pz: %f; rotx: %f; roty %f; rotz: %f", px_vec[i], py_vec[i], pz_vec[i], rotx_vec[i], roty_vec[i], rotz_vec[i]);
        }  
        pxStat_ = getStatistic(pxVec_);
        pyStat_ = getStatistic(pyVec_);
        pzStat_ = getStatistic(pzVec_);
        rotxStat_ = getStatistic(rotzVec_);
        rotyStat_ = getStatistic(rotyVec_);
        rotzStat_ = getStatistic(rotxVec_);
    } 

    Eigen::Vector3d getRPY(const vpHomogeneousMatrix& M, bool output=false, bool degree=false) 
    {
        // *** Get Rotation
        vpRotationMatrix R;
        M.extract(R);

        Eigen::Matrix3d rotation_matrix;
        for (unsigned int ii = 0; ii < R.getRows(); ii++) {
            for (unsigned int jj = 0; jj < R.getCols(); jj++) {
                rotation_matrix(ii, jj) = R[ii][jj];
            }
        }
        // *** Converting the rotation matrix to Euler angle
        // ZYX order intrinsic -> extrinsic (unitXYZ): roll around x axis, then around y axis pitch, finally around z axis yaw, 
        // 0 for X ax,is, 1 for Y axis, 2 for Z axis        
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);    
        
        if (degree)
        {
            // *** Convert to degree
            euler_angles[0] = euler_angles[0] *180/M_PI;
            euler_angles[1] = euler_angles[1] *180/M_PI;
            euler_angles[2] = euler_angles[2] *180/M_PI;
        }

        if (output) 
        {
            std::stringstream ss; 
            ss << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose();
            ROS_INFO("%s", ss.str().c_str());
        }        
        return euler_angles;
    }   

    void printHomogeneousMatrix(const vpHomogeneousMatrix M)
    {
        int precision = 3;
        std::stringstream s0, s1, s2, s3;
        s0.precision(precision);
        s1.precision(precision);
        s2.precision(precision);
        s3.precision(precision);

        s0 << "[" << M[0][0] << "; " << M[0][1] << "; " <<M[0][2]<< "; " << M[0][3] << "]";
        s1 << "[" << M[1][0] << "; " << M[1][1] << "; " <<M[1][2]<< "; " << M[1][3] << "]";
        s2 << "[" << M[2][0] << "; " << M[2][1] << "; " <<M[2][2]<< "; " << M[2][3] << "]";
        s3 << "[" << M[3][0] << "; " << M[3][1] << "; " <<M[3][2]<< "; " << M[3][3] << "]";
        ROS_INFO("\n %s\n %s\n %s\n %s\n", s0.str().c_str(), s1.str().c_str(), s2.str().c_str(), s3.str().c_str());   
    }  
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_node");
    ROS_INFO("Detection node has been started");

    CamDetection detection("detection");
    detection.executeCB();

    ros::shutdown();
    return 0;
}

