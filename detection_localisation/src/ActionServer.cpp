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
        }
        if (name == "SIFT" || "SURF"){
            detectorName_ = name;
            extractorName_ = name;
            matcherName_ = "FlannBased"; // "BruteForce"
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
        // STATUS = STATUS_TEST;
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
                    
                    // Check file for object model
                    if (vpIoTools::checkFilename(goal->file_3Dmodel)) {
                        modelFile_ = goal->file_3Dmodel;
                    } else {
                        ROS_ERROR("Can not open object modelfile(.cao or .wrl): %s", (goal->file_3Dmodel).c_str() );
                        STATUS = STATUS_ABORTED;                        
                    }

                    // *** Check file for learning data
                    learningData_ = goal->file_learning_data;                   
                    if (!vpIoTools::checkFilename(learningData_)) {
                        ROS_ERROR("Can not open learning data: %s", (learningData_).c_str());            
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

                    // *** Tracker settings
                    tracker.setOgreVisibilityTest(false);
                    tracker.setDisplayFeatures(true);
                    tracker.loadModel(modelFile_);
                    if (vpIoTools::checkFilename(goal->file_tracker_config)) {
                        tracker.loadConfigFile(goal->file_tracker_config); // .xml file
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
                    keypoint.loadLearningData(learningData_, binMode); 
                    if (vpIoTools::checkFilename(goal->file_detection_config)){                    
                        keypoint.loadConfigFile(goal->file_detection_config);
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
                    if (matches > MACTHES_THRESHOLD) 
                    {
                        ROS_INFO("Matches: %i", matches);                       
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
                            ROS_INFO("PUBLISH FEEDBACK state %i (Searching)", feedback_.state );
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
                        tracker.setPose(I_gray, cMo_);   
                        cMoVec_.push_back(cMo_);
                        frame++;
                        // ROS_INFO("Frame %i", frame);
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

            // ROS_INFO("px: %f; py: %f; pz: %f; rotx: %f; roty %f; rotz: %f", px_vec[i], py_vec[i], pz_vec[i], rotx_vec[i], roty_vec[i], rotz_vec[i]);
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_server");
    ROS_INFO("Detection node has been started");

    CamDetectionAction detection("detection");

    ros::spin();
    return 0;
}
