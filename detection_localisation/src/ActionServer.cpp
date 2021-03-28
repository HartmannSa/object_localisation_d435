// #define VISP_HAVE_REALSENSE2 true
// #define VISP_HAVE_OPENCV true
// #if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OPENCV)

#include <detection_localisation/CamDetectionAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
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

typedef actionlib::SimpleActionServer<detection_localisation::CamDetectionAction> Server; 

class CamDetectionAction
{
protected:

    ros::NodeHandle n_;
    ros::Subscriber sub_;
    actionlib::SimpleActionServer<detection_localisation::CamDetectionAction> server_;    
    detection_localisation::CamDetectionResult result_;
    detection_localisation::CamDetectionFeedback feedback_;
    int STATE_SEARCHING = 0;
    int STATE_FINISH = 1;
    int STATE_FOUND_MATCH = 2;
    int STATE_REFINE = 3;
    
    int STATUS;
    int STATUS_CHECK_DATA = 0;
    int STATUS_INIT = 1;
    int STATUS_IDLE = 2;
    int STATUS_TEST = 3;
    int STATUS_SEARCHING = 5;     
    int STATUS_POSE_REFINEMENT = 10;
    int STATUS_DISPLAY_MATCH = 20;
    int STATUS_PUBLISH_FEEDBACK = 99;
    int STATUS_END_FRAME_PROCESSING = 100;
    int STATUS_END_DETECTION = 500;
    int STATUS_PREEMPTED = 990;
    int STATUS_ABORTED = 991;
    int STATUS_EXIT = 999;  

    int MACTHES_THRESHOLD = 40; 
    
    std::string objectName_;
    std::string pathPackage_;       // path to package detection_localisation (starting in catkin_ws)
    std::string pathObjectFolder_; 
    std::string modelFile_;
    std::string learningData_;

    // Camera Settings
    vpRealSense2 realsense_;
    vpDisplayOpenCV d_g, d_d;
    vpCameraParameters camColor_, camDepth_; 
    vpImage<vpRGBa> I_color; 
    vpImage<unsigned char> I_gray, I_depth, I_matches, I_train;
    vpImage<uint16_t> I_depth_raw; 
    vpHomogeneousMatrix cMo_;
    vpHomogeneousMatrix cMo_prev;
    vpHomogeneousMatrix cMo_diff;
    vpHomogeneousMatrix depth_M_color; 
    std::vector<vpHomogeneousMatrix> cMoVec_; 

    std::string detectorName_;
    std::string extractorName_;
    std::string matcherName_;  
    std::string configurationFile_;
    
    bool moving_;
    bool verbose_;
    bool success_;
    int frameThreshold;

    double start, looptime, fps_measured;
    std::vector<double> times_vec;
    
    
public:
    CamDetectionAction(std::string name):
        server_(n_, name, boost::bind(&CamDetectionAction::executeCB, this, _1), false)
    {        
        server_.start();
        sub_ = n_.subscribe("/odom_comb", 1, &CamDetectionAction::analysisMovement, this);
        moving_ = false;         // In case topic isnt published
        ros::NodeHandle nh("~");
        nh.param<std::string>("path_package", pathPackage_, "src/object_localisation_d435/detection_localisation");
        nh.param<bool>("verbose", verbose_, true);
        nh.param<int>("frameThreshold", frameThreshold, 20);
        cMoVec_.assign(frameThreshold, cMo_);
    }

    ~CamDetectionAction(void) {}

    void analysisMovement(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double t = 0.01;   // threshold
        double x = msg->twist.twist.linear.x;
        double z = msg->twist.twist.angular.z;
        bool moving_old = moving_;

        moving_ = (abs(x) > t || abs(z) > t) ? true : false;

        if (moving_ != moving_old && verbose_) 
        {
            if (moving_) 
                ROS_INFO("Robot starts to move");
            else
                ROS_INFO("Robot stops moving");
        }
                    
    }

    bool startCamera(int width, int height, int fps)
    {   
        // *** Configuration
        rs2::config config;        
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        depth_M_color.buildFrom(vpTranslationVector(-0.015,0,0), vpQuaternionVector(0,0,0,1));
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
        I_color.resize(height, width);
        I_gray.resize(height, width);
        I_depth.resize(height, width);
        I_depth_raw.resize(height, width);
        I_matches.resize(height, 2*width); 
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
    
    void getFrame(){
        looptime = vpTime::measureTimeMs() - start;
        times_vec.push_back(looptime);
        fps_measured = 1/(looptime/1000);

        start = vpTime::measureTimeMs();
        realsense_.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);
        vpImageConvert::convert(I_color, I_gray);
        vpDisplay::display(I_gray);
        vpDisplay::displayText(I_gray, 10, 10, "Detection and localization in process...", vpColor::red);
        vpDisplay::displayText(I_gray, 30, 10, "FPS measured: " + std::to_string(fps_measured) , vpColor::red);

    }

    void executeCB(const detection_localisation::CamDetectionGoalConstPtr &goal)
    {
        double start = vpTime::measureTimeSecond();
        STATUS = STATUS_CHECK_DATA;
        STATUS = STATUS_TEST;
        bool proceed = true;        
        int frame = 0;
        vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
        vpKeyPoint keypoint;
        unsigned int matches;

        while (server_.isActive() && proceed && ros::ok()) {
            // *** Check if preempt is requested
            if (server_.isPreemptRequested())
                STATUS = STATUS_PREEMPTED;

            switch (STATUS)
            {
                case 0:     // STATUS_CHECK_DATA
                {
                    ROS_INFO("STATUS: CHECK DATA AND REQUEST");
                    STATUS = STATUS_INIT; 
                    
                    // *** Check if name of detection goal is given
                    if (goal->object_name != "" ) {
                        objectName_ = goal->object_name;
                    } else {
                        ROS_ERROR("No objectname given in goal." );
                        STATUS = STATUS_ABORTED; 
                    }              
                    
                    // Check files for object model
                    pathObjectFolder_ = pathPackage_ + "/model/" + objectName_ + "/";                    
                    if (vpIoTools::checkFilename(pathObjectFolder_ + objectName_ + ".cao")) {
                        modelFile_ = pathObjectFolder_ + objectName_ + ".cao";
                    } else if (vpIoTools::checkFilename(pathObjectFolder_ + objectName_ + ".wrl")) {
                        modelFile_ = pathObjectFolder_ + objectName_ + ".wrl";
                    } else {
                        ROS_ERROR("Can not open object modelfile(.cao or .wrl): %s", (pathObjectFolder_ + objectName_).c_str() );
                        STATUS = STATUS_ABORTED;                        
                    }

                    // *** Check file for learning data
                    learningData_ = goal->learning_data;                   
                    if (!vpIoTools::checkFilename(pathObjectFolder_ + learningData_)) {
                        ROS_ERROR("Can not open learning data: %s", (pathObjectFolder_+learningData_).c_str());            
                        STATUS = STATUS_ABORTED;
                    }              

                    // *** Check Camera start
                    int width = 640, height = 480, fps = 30;                     
                    if (!startCamera(width, height, fps)) {
                        ROS_ERROR("Can not open Camera");
                        STATUS = STATUS_ABORTED;
                    } 
                    
                    ROS_INFO("Valid detection request and input data.");  
                    break;
                }
                case 1:     // STATUS_INIT
                {
                    ROS_INFO("STATUS: INIT");
                    success_ = false;
                    // *** Display settings                    
                    unsigned int _posx = 100, _posy = 50;        
                    d_g.init(I_gray, _posx, _posy, "Color stream");
                    // d_d.init(       I_depth, _posx + I_gray.getWidth()+10,  _posy,                          "Depth stream");
                    // *** Read train image as reference
                    // vpImageIo::read(I_train, pathPackage_ + "/model/" + goal->object_name + "/" + goal->object_name + "_gray.jpeg");
                    // ROS_INFO("Reference keypoints = %i", keypoint.buildReference(I_train) );                    

                    // *** Tracker settings
                    tracker.setOgreVisibilityTest(false);
                    tracker.setDisplayFeatures(true);
                    tracker.loadModel(modelFile_);
                    if (vpIoTools::checkFilename(pathObjectFolder_ + objectName_ + ".xml")) {
                        tracker.loadConfigFile(pathObjectFolder_ + objectName_ + ".xml");
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
                        tracker.setAngleAppear(vpMath::rad(70));
                        tracker.setAngleDisappear(vpMath::rad(80));
                        tracker.setNearClippingDistance(0.1);
                        tracker.setFarClippingDistance(100.0);
                        tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
                    }
                    
                    // *** Keypoint Settings (detector, extractor, matcher, learning data)                   
                    setDetectionSettings("SURF", keypoint);
                    bool binMode = ( learningData_.rfind(".bin") != std::string::npos );
                    keypoint.loadLearningData(pathObjectFolder_ + learningData_, binMode); 
                    if (vpIoTools::checkFilename(pathObjectFolder_ + configurationFile_)){                    
                        keypoint.loadConfigFile(pathObjectFolder_ + configurationFile_);
                    } else {            
                        keypoint.setMatchingRatioThreshold(0.8);
                        keypoint.setUseRansacVVS(true);
                        keypoint.setUseRansacConsensusPercentage(true);
                        keypoint.setRansacConsensusPercentage(20.0);
                        keypoint.setRansacIteration(200);
                        keypoint.setRansacThreshold(0.005);
                    }                     
                    STATUS = STATUS_SEARCHING;
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
                    // *** Test Mean, Stdev, Median
                    // std::vector<double> test;
                    // test.push_back(0.0);
                    // test.push_back(0.2);
                    // test.push_back(0.0);
                    // test.push_back(0.1);
                    // test.push_back(0.3);                    
                    // ROS_INFO("Mean: %f und Stdev: %f",vpMath::getMean(test), vpMath::getStdev(test));
                    // ROS_INFO("Median Soll: 0.1   Ist: %f",getMedian(test));
                    // test.push_back(0.0);
                    // test.push_back(0.4);
                    // test.push_back(0.7);
                    // test.push_back(0.0);
                    // ROS_INFO("Median Soll: 0.1   Ist: %f",getMedian(test));
                    // test.push_back(0.9);
                    // ROS_INFO("Median gerade Vec.size -> /2 aufrunden ->  Ist: %f",getMedian(test));

                    // *** Check Angles from H
                    std::vector<double> v{0.6568678029550146,-0.046966760660189066,0.7525287766825592,0,-0.0784101145291276,-0.9968920972698805,0.006224728234767274,0,0.7499120732530682,-0.0630959059174174,-0.6585216693820678,0,0.015625173638133094,0.3034115542624131,0.4425090445578807,1};
                    vpHomogeneousMatrix H, HT;
                    for (int col=0; col<4;col++)
                    {
                        for (int row=0; row<4;row++)
                        {
                            H[col][row] = v[row+col*4];
                        }
                    }
                    printHomogeneousMatrix(H);                
                    getRPY(H, true, true);

                    for (int row=0; row<4;row++)
                    {
                        for (int col=0; col<4;col++)
                        {
                            HT[col][row] = v[col+row*4];
                        }
                    }
                    printHomogeneousMatrix(HT);
                    getRPY(HT, true, true);
                    
                    STATUS = STATUS_IDLE;
                    break;
                }

                case 5:     // STATUS_SEARCHING
                {
                    // ROS_INFO("STATUS: SEARCHING");
                    getFrame();                                    
                                        
                    double error, elapsedTime;
                    keypoint.matchPoint(I_gray, camColor_, cMo_, error, elapsedTime);
                    matches = keypoint.getMatchedPointNumber();
                    ROS_INFO("Matches: %i", matches);
                    if (matches > MACTHES_THRESHOLD) 
                    {                       
                        tracker.setPose(I_gray, cMo_);
                        feedback_.estimated_pose = createPosesStamped(cMo_);
                        feedback_.state = STATE_FOUND_MATCH;
                        server_.publishFeedback(feedback_);
                        ROS_INFO("PUBLISH FEEDBACK state %i", feedback_.state );

                        tracker.display(I_gray, cMo_, camColor_, vpColor::red, 2);                   
                        vpDisplay::displayFrame(I_gray, cMo_, camColor_, 0.025, vpColor::none, 3);

                        if (!moving_){
                            STATUS = STATUS_POSE_REFINEMENT;
                            cMoVec_.clear();
                            frame = 0;
                        }

                    } else {
                        feedback_.state = STATE_SEARCHING;
                        if (!moving_) {
                            server_.publishFeedback(feedback_);
                            ROS_INFO("PUBLISH FEEDBACK state %i", feedback_.state );
                        }
                    }
                    vpDisplay::flush(I_gray);                    

                    break; 
                }    
                case 10:    // STATUS_POSE_REFINEMENT: case robot stops moving -> Start to calculate finale cMo_                                    
                {    
                    ROS_INFO("STATUS: POSE REFINEMENT");
                    feedback_.state = STATE_REFINE;
                    getFrame();
                    
                    double error, elapsedTime;
                    keypoint.matchPoint(I_gray, camColor_, cMo_, error, elapsedTime);
                    matches = keypoint.getMatchedPointNumber();
                    ROS_INFO("Matches: %i", matches);
                    if (matches > MACTHES_THRESHOLD) 
                    {
                        unsigned int matches;
                        matches = keypoint.getMatchedPointNumber();
                        ROS_INFO("Matches: %i", matches);
                        tracker.setPose(I_gray, cMo_);   
                        cMoVec_.push_back(cMo_);
                        frame++;
                        ROS_INFO("Frame %i", frame);
                        if (frame >= frameThreshold)
                        {
                            // *** Compute resulting cMo_ from cMoVec_                        
                            // computeResult(cMo_, result_.translation_stdev, result_.rotation_stdev);  
                            computeResult(cMo_);                                                        
                            // ROS_INFO("STDV Translation: %f %f %f", result_.translation_stdev.x, result_.translation_stdev.y, result_.translation_stdev.z);
                            // ROS_INFO("STDV Rotation: %f %f %f", result_.rotation_stdev.x, result_.rotation_stdev.y, result_.rotation_stdev.z);
                            success_ = true;
                            feedback_.state = STATE_FINISH;
                            STATUS = STATUS_END_DETECTION;                        
                        }

                        tracker.display(I_gray, cMo_, camColor_, vpColor::red, 2);                   
                        vpDisplay::displayFrame(I_gray, cMo_, camColor_, 0.025, vpColor::none, 3);
                        
                    } else {
                        STATUS = STATUS_SEARCHING;
                    }
                    vpDisplay::flush(I_gray);
                    server_.publishFeedback(feedback_);
                    ROS_INFO("PUBLISH FEEDBACK state %i", feedback_.state );
                    break;
                }                   
                case 500:   // STATUS_END_DETECTION
                {
                    ROS_INFO("STATUS: EXECUTION FINISHED!");
                    if (verbose_) {
                        double detectionTime = vpTime::measureTimeSecond() - start;                        
                        ROS_INFO("Processing time: %.2f min", detectionTime/60);
                    } 
                    // saveData();
                    if (success_) {
                        result_.object_pose = createPosesStamped(cMo_); 
                        result_.angles = getAngles(cMo_);
                        server_.setSucceeded(result_);
                        ROS_INFO("SEARCH SUCCEEDED: FOUND %s!", objectName_.c_str());
                    } else {
                        STATUS = STATUS_EXIT;
                    }
                    break;
                }
                case 990:   // STATUS_PREEMPTEDF 
                {
                    ROS_INFO("STATUS: PREEMPTED (Request canceld)");
                    server_.setPreempted();
                    STATUS = STATUS_EXIT;
                    break;
                }
                case 991:   // STATUS_ABORTED
                {
                    ROS_ERROR("STATUS: ABORTED GOAL %s", objectName_.c_str());
                    server_.setAborted();                    
                    STATUS = STATUS_EXIT;
                    break;
                }
                case 999:   // STATUS_EXIT LOOP
                {
                    ROS_INFO("STATUS: EXIT CALLBACK");
                    proceed = false;  
                    break;  
                }     
            }

        }

        // *** Send Feedback only when cMo_ is different to cMo_prev             
        // cMo.print(); std::cout << std::endl;
        // printHomogeneousMatrix(cMo);
        // printRPY(cMo_);
        // vpMatrix M;
        // vpMatrix::sub2Matrices(cMo_prev, cMo_, M);
        // double e = 0.1;
        // if (M.getMaxValue() > e){                    
        //     printMatrix(M); 
        //     ROS_INFO("Max difference is: %.3f", M.getMaxValue());
        //     sendFeedback(cMo);
        // }                
        // cMo_prev = cMo_;

        // *** Print Feedback
        // if (feedback_.state != feedback_previous.state) {ROS_INFO("Feedback state switched to %i", feedback_.state );}              
        // feedback_previous = feedback_;
        
        // I_matches.insert(I_gray, vpImagePoint(0, I_train.getWidth()));
        // vpDisplay::display(I_matches);
        // vpDisplay::displayLine(I_matches, vpImagePoint(0, I_train.getWidth()), 
        //             vpImagePoint(I_train.getHeight(), I_train.getWidth()), vpColor::white, 2);              
        // if (use_depth) {
        //     vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
        //     vpDisplay::display(I_depth);
        //     vpDisplay::flush(I_depth);
        // }
        // unsigned int nbMatch = keypoint.matchPoint(I_gray);     // I_gray is matched with reference image
        // ROS_INFO("Matches = %i", nbMatch);
        // vpImagePoint iPref, iPcur;
        // for (unsigned int i = 0; i < nbMatch; i++) {
        //     keypoint.getMatchedPoints(i, iPref, iPcur);
        //     vpDisplay::displayLine(I_matches, iPref, iPcur + vpImagePoint(0, I_train.getWidth()), vpColor::green);
        // }
        // vpDisplay::flush(I_matches);
    }

    void saveData()
    {
        std::string filename = pathPackage_ + "/DetectionDataLog.txt";
        std::ofstream f;
        f.open(filename.c_str());

        f << "cMoVec_" << std::endl;
        for (int i = 0; i < cMoVec_.size(); i++)
        {
            f << cMoVec_[i] << ";\n" << std::endl;
        }

        f << "\nCalculated cMo" << std::endl;
        f << cMo_ << std::endl;

        f << "\nresult object pose" << std::endl;
        f << result_.object_pose << std::endl;           

        f.close();
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

        // std::string s1;
        // s1 = "[%.2f; %.2f; %.2f; %.2f]", M[0][0], M[0][1], M[0][2], M[1][3];
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[0][0], M[0][1], M[0][2], M[0][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[1][0], M[1][1], M[1][2], M[1][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[2][0], M[2][1], M[2][2], M[2][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[3][0], M[3][1], M[3][2], M[3][3] );   
    }
    
    void printMatrix(const vpMatrix& M)
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

    void printMatrix(const Eigen::Matrix3d& M)
    {
        int precision = 3;
        std::stringstream s0, s1, s2, s3;
        s0.precision(precision);
        s1.precision(precision);
        s2.precision(precision);
        s3.precision(precision);

        s0 << "[" << M(0,0) << "; " << M(0,1) << "; " <<M(0,2)<< "]";
        s1 << "[" << M(1,0) << "; " << M(1,1) << "; " <<M(1,2)<< "]";
        s2 << "[" << M(2,0) << "; " << M(2,1) << "; " <<M(2,2)<< "]";

        ROS_INFO("\n %s\n %s\n %s\n", s0.str().c_str(), s1.str().c_str(), s2.str().c_str()); 
    }
    
    double getMedian(std::vector<double> vec)
    {
        double median;
        std::nth_element(vec.begin(), vec.begin()+vec.size()/2, vec.end());
        median = vec[vec.size()/2];
        return median;
    }

    void computeResult(vpHomogeneousMatrix& M)
    {
        std::vector<double> px_vec, py_vec, pz_vec, rotx_vec, roty_vec, rotz_vec;
        double rotx, roty, rotz;
        Eigen::Vector3d euler_angles;
        vpRotationMatrix R;

        ROS_INFO("cMoVec size is %zu", cMoVec_.size());
        for (int i = 0; i < cMoVec_.size(); i++)
        {
            px_vec.push_back(cMoVec_[i][0][3]);
            py_vec.push_back(cMoVec_[i][1][3]);
            pz_vec.push_back(cMoVec_[i][2][3]);

            euler_angles = getRPY(cMoVec_[i], false, true);
            rotz_vec.push_back(euler_angles[0]);    // gier = yaw
            roty_vec.push_back(euler_angles[1]);    // nick = pitch
            rotx_vec.push_back(euler_angles[2]);    // roll

            ROS_INFO("px: %f; py: %f; pz: %f; rotx: %f; roty %f; rotz: %f", px_vec[i], py_vec[i], pz_vec[i], rotx_vec[i], roty_vec[i], rotz_vec[i]);
        }  

        // *** Mean     
        // M[0][3] = vpMath::getMean(px_vec);
        // M[1][3] = vpMath::getMean(py_vec);
        // M[2][3] = vpMath::getMean(pz_vec);
        // rotz    = vpMath::getMean(rotz_vec)*(M_PI/180);    // Umwandlung in Radian für R.build
        // roty    = vpMath::getMean(roty_vec)*(M_PI/180);    
        // rotx    = vpMath::getMean(rotx_vec)*(M_PI/180);
        // R.buildFrom(vpRzyxVector(rotz, roty, rotx)); 
        // M.insert(R);   


        // Median
        // vpMath::getMedian()
        M[0][3] = getMedian(px_vec);
        M[1][3] = getMedian(py_vec);
        M[2][3] = getMedian(pz_vec);
        rotz    = getMedian(rotz_vec)*(M_PI/180);    // Umwandlung in Radian für R.build
        roty    = getMedian(roty_vec)*(M_PI/180);    
        rotx    = getMedian(rotx_vec)*(M_PI/180);
        R.buildFrom(vpRzyxVector(rotz, roty, rotx)); 
        M.insert(R);
        
        // Stdev
        result_.px_stdev = vpMath::getStdev(px_vec);
        result_.py_stdev = vpMath::getStdev(py_vec);
        result_.pz_stdev = vpMath::getStdev(pz_vec);
        result_.rotx_stdev = vpMath::getStdev(rotx_vec);
        result_.roty_stdev = vpMath::getStdev(roty_vec);
        result_.rotz_stdev = vpMath::getStdev(rotz_vec);
        // result_.rotx_stdev = vpMath::getStdev(rotx_vec);
        // ROS_INFO("rotx %f", result_.rotx_stdev);
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
        // printMatrix(rotation_matrix);

        // 2.1 Converting the rotation matrix to Euler angle
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
    
    geometry_msgs::Point getAngles(const vpHomogeneousMatrix& M) 
    {
        geometry_msgs::Point angles;
        Eigen::Vector3d vec;
        vec = getRPY(M, false, true);
        angles.x = vec[0];
        angles.y = vec[1];
        angles.z = vec[2];

        return angles;
    }

    geometry_msgs::PoseStamped createPosesStamped(const vpHomogeneousMatrix& M)
    {        
        vpQuaternionVector q;
        vpTranslationVector translation;
        geometry_msgs::PoseStamped poseStamped; 
        
        M.extract(q);
        M.extract(translation);

        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "camera_arm_color_optical_frame";

        poseStamped.pose.position.x = translation[0];
        poseStamped.pose.position.y = translation[1];
        poseStamped.pose.position.z = translation[2];
        poseStamped.pose.orientation.x = q.x();
        poseStamped.pose.orientation.y = q.y();
        poseStamped.pose.orientation.z = q.z();
        poseStamped.pose.orientation.w = q.w();
        // broadcast_transformation(parent_frame, child_frame, q, translation);
        return poseStamped;
    }


    
    std::string parent_frame, child_frame;
    std::string config_color, config_depth;
    std::string model_color, model_depth;
    std::string init_file;
    std::string learning_data;
    bool use_ogre;
    bool use_scanline;
    bool use_edges;
    bool use_klt;
    bool use_depth;
    bool learn;
    bool auto_init;
    bool display_projection_error;
    bool broadcast_transform;
    double proj_error_threshold;
 
};

void broadcast_transformation(const std::string parent_frame, const std::string child_frame, const vpQuaternionVector& q, const vpTranslationVector& translation) 
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;

  transformStamped.transform.translation.x = translation[0];
  transformStamped.transform.translation.y = translation[1];
  transformStamped.transform.translation.z = translation[2];
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}


// void execute(const detection_localisation::CamDetectionGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended here
// {    
//     q[0] = 0;
//     q[1] = 1;
//     q[2] = 0;
//     q[3] = 0;      
//     // if (broadcast_transform) {
//     //     ros::init(argc, argv, "cam_object_broadcaster");
//     // }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  TRACKER settings 
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     std::vector<int> trackerTypes;
//     if (use_edges && use_klt)
//         trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
//     else if (use_edges)
//         trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER );
//     else if (use_klt)
//         trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER);
//     if (use_depth)
//         trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);    
//     vpMbGenericTracker tracker(trackerTypes);
//     // In Case 2 camera frames are used (color and depth) maps need to be defined
//     vpHomogeneousMatrix depth_M_color = realsense_.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
//     std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
//     std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
//     std::map<std::string, std::string> mapOfInitFiles;
//     std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
//     std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
//     std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;    
//     // Set tracker configurations depending of which frames and features are used
//     if ((use_edges || use_klt) && use_depth) {
//         tracker.loadConfigFile(config_color, config_depth);
//         tracker.loadModel(model_color, model_depth);
//         std::cout << "Sensor internal depth_M_color: \n" << depth_M_color << std::endl;
//         mapOfCameraTransformations["Camera2"] = depth_M_color;
//         tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
//         mapOfImages["Camera1"] = &I_gray;
//         mapOfImages["Camera2"] = &I_depth;
//         mapOfInitFiles["Camera1"] = init_file;
//         tracker.setCameraParameters(camColor_, camDepth_);//     }
//     else if (use_edges || use_klt) {
//         tracker.loadConfigFile(config_color);
//         tracker.loadModel(model_color);
//         tracker.setCameraParameters(camColor_);
//     }
//     else if (use_depth) {
//         tracker.loadConfigFile(config_depth);
//         tracker.loadModel(model_depth);
//         tracker.setCameraParameters(camDepth_);
//     }
//     tracker.setDisplayFeatures(true);
//     tracker.setOgreVisibilityTest(use_ogre);
//     tracker.setScanLineVisibilityTest(use_scanline);
//     tracker.setProjectionErrorComputation(true);
//     tracker.setProjectionErrorDisplay(display_projection_error);    
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  DETECTOR, EXTRACTOR, MATCHER settings
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     #if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)) || \
//         (VISP_HAVE_OPENCV_VERSION >= 0x030411 && CV_MAJOR_VERSION < 4) || (VISP_HAVE_OPENCV_VERSION >= 0x040400)
//         std::string detectorName = "SIFT";
//         std::string extractorName = "SIFT";
//         std::string matcherName = "BruteForce";
//     #else
//         std::string detectorName = "FAST";
//         std::string extractorName = "ORB";
//         std::string matcherName = "BruteForce-Hamming";
//     #endif
//     vpKeyPoint keypoint;
//     if (learn || auto_init) 
//     {
//         keypoint.setDetector(detectorName);
//         keypoint.setExtractor(extractorName);
//         keypoint.setMatcher(matcherName);
//         // Set ORB Level Parameter
//         #if !(defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
//             #if (VISP_HAVE_OPENCV_VERSION < 0x030000)
//                 keypoint.setDetectorParameter("ORB", "nLevels", 1);
//             #else
//                 cv::Ptr<cv::ORB> orb_detector = keypoint.getDetector("ORB").dynamicCast<cv::ORB>();
//                 if (orb_detector) {
//                     orb_detector->setNLevels(1);
//                 }
//             #endif
//         #endif
//     }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------    
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  TRACKER Initialisierung (auto init oder USER CLICKS)
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     if (auto_init) // 2.option: file data-learned.bin aleready exists, you already learned the object, same initial pose
//     {
//         if (!vpIoTools::checkFilename(learning_data)) {
//             std::cout << "Cannot enable auto detection. Learning file \"" << learning_data << "\" doesn't exist" << std::endl;
//             //return EXIT_FAILURE;
//             as->setPreempted();
//         }
//         keypoint.loadLearningData(learning_data, true);
//     } else //1.option: Init tracker with clicking the 4 points in the first frame
//     {
//         if ((use_edges || use_klt) && use_depth)
//             tracker.initClick(mapOfImages, mapOfInitFiles, true);
//         else if (use_edges || use_klt)
//             tracker.initClick(I_gray, init_file, true);
//         else if (use_depth)
//             tracker.initClick(I_depth, init_file, true);
//         if (learn)  // 1.b: save initialisation in the file data-learned,bin, so that auto_init can be performed next time
//             vpIoTools::makeDirectory(vpIoTools::getParent(learning_data));
//     }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------    
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  Declare some needed variables
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     bool quit = false;
//     bool learn_position = false;
//     bool run_auto_init = false;
//     if (auto_init) { run_auto_init = true;}    
//     std::vector<vpColVector> pointcloud;
//     std::vector<double> times_vec;
//     double loop_t = 0;
//     int learn_id = 1;
//     vpHomogeneousMatrix cMo;
//     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  MAIN ALGORITHM - LOOP
//     // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     try {
//         while (!quit) {
//             double t = vpTime::measureTimeMs();
//             bool tracking_failed = false;
//             // --------------------------------------------------------------------------------------------------------------------------
//             //  AQUIRE IMAGES and update tracker input data
//             // --------------------------------------------------------------------------------------------------------------------------
//             realsense_.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, &pointcloud, NULL, NULL);
//             if (use_edges || use_klt || run_auto_init) {
//                 vpImageConvert::convert(I_color, I_gray);
//                 vpDisplay::display(I_gray);
//             }
//             if (use_depth) {
//                 vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
//                 vpDisplay::display(I_depth);
//             }
//             if ((use_edges || use_klt) && use_depth) {
//                 mapOfImages["Camera1"] = &I_gray;
//                 mapOfPointclouds["Camera2"] = &pointcloud;
//                 mapOfWidths["Camera2"] = width;
//                 mapOfHeights["Camera2"] = height;
//             } else if (use_edges || use_klt) {
//                 mapOfImages["Camera"] = &I_gray;
//             } else if (use_depth) {
//                 mapOfPointclouds["Camera"] = &pointcloud;
//                 mapOfWidths["Camera"] = width;
//                 mapOfHeights["Camera"] = height;
//             }
//             // Run auto initialization from learned data
//             if (run_auto_init) {
//                 if (keypoint.matchPoint(I_gray, camColor_, cMo)) // Check if auto initialisation was good via keypoint matching
//                 {
//                     std::cout << "Auto init succeed" << std::endl;
//                     if ((use_edges || use_klt) && use_depth) {
//                         mapOfCameraPoses["Camera1"] = cMo;
//                         mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
//                         tracker.initFromPose(mapOfImages, mapOfCameraPoses);
//                     } else if (use_edges || use_klt) {
//                         tracker.initFromPose(I_gray, cMo);
//                     } else if (use_depth) {
//                         tracker.initFromPose(I_depth, depth_M_color*cMo);
//                     }
//                 } else {
//                     std::cout << "Auto init did NOT succeed" << std::endl;
//                     if (use_edges || use_klt) {
//                         vpDisplay::flush(I_gray);
//                     }
//                     if (use_depth) {
//                         vpDisplay::flush(I_depth);
//                     }
//                     continue;
//                 }
//             }
//             // --------------------------------------------------------------------------------------------------------------------------
//             // Run the tracker
//             // --------------------------------------------------------------------------------------------------------------------------
//             try {
//                 if (run_auto_init) {
//                     // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
//                     tracker.setDisplayFeatures(false);
//                     run_auto_init = false;
//                 }
//                 if ((use_edges || use_klt) && use_depth) {
//                     tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
//                 } else if (use_edges || use_klt) {
//                     tracker.track(I_gray);
//                 } else if (use_depth) {
//                     tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
//                 }
//             } catch (const vpException &e) {
//                 std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
//                 tracking_failed = true;
//                 if (auto_init) {
//                     std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
//                     run_auto_init = true;
//                 }
//             }            
//             // --------------------------------------------------------------------------------------------------------------------------
//             // CHECK TRACKING ERRORS
//             // --------------------------------------------------------------------------------------------------------------------------
//             double proj_error = 0;
//             if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
//                 proj_error = tracker.getProjectionError();
//             } else {
//                 proj_error = tracker.computeCurrentProjectionError(I_gray, cMo, camColor_);
//             }
//             if (auto_init && proj_error > proj_error_threshold) {
//                 std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
//                 run_auto_init = true;
//                 tracking_failed = true;
//             }
//             // --------------------------------------------------------------------------------------------------------------------------
//             // Get and Send Transformation
//             // --------------------------------------------------------------------------------------------------------------------------
//             if (broadcast_transform){
//                 // Get object pose
//                 cMo = tracker.getPose();
//                 // std::cout << "cMo " << cMo << "\n";
//                 // std::cout << "depth_M_color -> " << depth_M_color << "\n";
//                 // 2.1 Converting the rotation matrix to Euler angle
//                 // ZYX order, that is, roll around the x axis, then around the y axis pitch, and finally around the z axis yaw, 0 for the X axis, 1 for the Y axis, 2 for the Z axis
//                 Eigen::Matrix3d rotation_matrix;
//                 vpRotationMatrix R;
//                 cMo.extract(R);
//                 for (unsigned int ii = 0; ii < R.getRows(); ii++) {
//                     for (unsigned int jj = 0; jj < R.getCols(); jj++) {
//                         rotation_matrix(ii, jj) = R[ii][jj];
//                     }
//                 }
//                 Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); 
//                 // std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << std::endl;
//                 cMo.extract(q);
//                 cMo.extract(translation);
//                 broadcast_transformation(parent_frame, child_frame, q, translation);
//             }
//             // --------------------------------------------------------------------------------------------------------------------------
//             // --------------------------------------------------------------------------------------------------------------------------
//             // DISPLAY TRACKING RESULTS
//             // --------------------------------------------------------------------------------------------------------------------------
//             if (!tracking_failed) {
//                 tracker.setDisplayFeatures(true);
//                 if ((use_edges || use_klt) && use_depth) {
//                     tracker.display(I_gray, I_depth, cMo, depth_M_color*cMo, camColor_, camDepth_, vpColor::red, 3);
//                     vpDisplay::displayFrame(I_gray, cMo, camColor_, 0.05, vpColor::none, 3);
//                     vpDisplay::displayFrame(I_depth, depth_M_color*cMo, camDepth_, 0.05, vpColor::none, 3);
//                 } else if (use_edges || use_klt) {
//                     tracker.display(I_gray, cMo, camColor_, vpColor::red, 3);
//                     vpDisplay::displayFrame(I_gray, cMo, camColor_, 0.05, vpColor::none, 3);
//                 } else if (use_depth) {
//                     tracker.display(I_depth, cMo, camDepth_, vpColor::red, 3);
//                     vpDisplay::displayFrame(I_depth, cMo, camDepth_, 0.05, vpColor::none, 3);
//                 }
//                 std::stringstream ss;
//                 ss << "Nb features: " << tracker.getError().size();
//                 vpDisplay::displayText(I_gray, I_gray.getHeight() - 50, 20, ss.str(), vpColor::red);       
//                 ss << "Features: edges " << tracker.getNbFeaturesEdge()
//                     << ", klt " << tracker.getNbFeaturesKlt()
//                     << ", depth " << tracker.getNbFeaturesDepthDense();
//                 vpDisplay::displayText(I_gray, I_gray.getHeight() - 30, 20, ss.str(), vpColor::red);
//             }
//             std::stringstream ss;
//             ss << "Loop time: " << loop_t << " ms";
//             // --------------------------------------------------------------------------------------------------------------------------
//             // DISPLAY AND GET USER INPUT -> quit, learn, autoinit
//             // --------------------------------------------------------------------------------------------------------------------------
//             vpMouseButton::vpMouseButtonType button;
//             if (use_edges || use_klt) {
//                 vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
//                 if (learn)
//                     vpDisplay::displayText(I_gray, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
//                 else if (auto_init)
//                     vpDisplay::displayText(I_gray, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
//                 else
//                     vpDisplay::displayText(I_gray, 35, 20, "Right click: quit", vpColor::red);
//                 vpDisplay::flush(I_gray);
//                 if (vpDisplay::getClick(I_gray, button, false)) {
//                     if (button == vpMouseButton::button3) {
//                         quit = true;
//                     } else if (button == vpMouseButton::button1 && learn) {
//                         learn_position = true;
//                     } else if (button == vpMouseButton::button1 && auto_init && !learn) {
//                         run_auto_init = true;
//                     }
//                 }
//             }
//             if (use_depth) {
//                 vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);
//                 vpDisplay::displayText(I_depth, 40, 20, "Click to quit", vpColor::red);
//                 vpDisplay::flush(I_depth);
//                 if (vpDisplay::getClick(I_depth, false)) {
//                     quit = true;
//                 }
//             }            
//             loop_t = vpTime::measureTimeMs() - t;
//             times_vec.push_back(loop_t);
//         }
//     } catch (const vpException &e) {
//         std::cout << "Catch an exception: " << e.what() << std::endl;
//     }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     if (!times_vec.empty()) {
//         std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
//                 << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;
//     }
//     as->setSucceeded();
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_server");
    ROS_INFO("Detection node has been started");

    // Server server(nh, "detect_object", boost::bind(&execute, _1, &server), false);
    // server.start();

    CamDetectionAction detection("detection");

    ros::spin();
    return 0;
}

// #elif defined(VISP_HAVE_REALSENSE2)
//     int main() {
//         std::cout << "Install OpenCV 3rd party, configure and build ViSP again to use this example" << std::endl;
//         return 0;
//     }
// #else
//     int main() {
//         std::cout << "Install librealsense2 3rd party, configure and build ViSP again to use this example" << std::endl;
//         return 0;
//     }
// #endif 