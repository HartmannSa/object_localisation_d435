
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>         // in createTrainImages to create model folder

#include <unistd.h>
#include <iostream>
#include <std_msgs/String.h>

#include <sstream>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>


template<class T>
std::string toString(const T &value) {
    std::ostringstream os;
    os << value;
    return os.str();
}


std::vector<cv::Mat> getHorizontalStack(int n, std::string path,  std::string name )
{
    std::vector<cv::Mat> matrices;
    int i = 1;
    while (ros::ok() && i <= n) 
    { 
        cv::Mat img;  
        std::string filename = path + name + "/" + name + std::to_string(i) + "_color.jpg";          
        img = cv::imread(filename);
        if (img.empty()) { ROS_ERROR("Could not open or find the image: %s", filename.c_str());}        

        if (i == 1) {
            ROS_INFO("Start loading with image %s", filename.c_str() );
            cv::putText(img, name, cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.0, cv::Scalar(255,255,255), 2, cv:: LINE_AA); 
        } else {
            cv::putText(img, std::to_string(i), cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.0, cv::Scalar(255,255,255), 2, cv:: LINE_AA); 
        } 

        matrices.push_back(img);
        i++;               
    } 
    return matrices;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "display_images"); 
    ROS_INFO("Node started");

    std::string pathModel_; 
    int numberImages_;
    ros::NodeHandle nh("~");
    nh.param<std::string>("path", pathModel_, "");                                            // path to package, e.g learn_object/model/
    nh.param<int>("numberOfImages", numberImages_, 3);
    ROS_INFO("Node to display images started" );
    
    std::vector<std::string> object_name;
    object_name.push_back("Goesser_27_56_");
    object_name.push_back("Goesser_35_77_");
    object_name.push_back("Goesser_44_106_");
    object_name.push_back("Goesser_52_141_");
    object_name.push_back("Goesser_35_77_");
    object_name.push_back("Goesser_35_98_");
    object_name.push_back("Goesser_35_119_");
    object_name.push_back("Goesser_35_140_");

    
    std::vector<cv::Mat> matricesV;
    int i = 0;
    while (ros::ok() && i < object_name.size()) 
    {   
        cv::Mat imgStack;
        std::vector<cv::Mat> matrices;      
        matrices = getHorizontalStack(numberImages_, pathModel_, object_name[i]);
        cv::hconcat(matrices, imgStack);
        matricesV.push_back(imgStack);
        ROS_INFO("i: %d", i);
        i++;
    }
    cv::Mat imgResult;
    cv::vconcat(matricesV, imgResult);
    // cv::resize(imgResult, imgResult, cv::Size(imgResult.cols/2, imgResult.rows/2));
   
    cv::String windowName = "ImageStack ";     
    cv::namedWindow(windowName, cv::WINDOW_NORMAL );   //cv::WINDOW_AUTOSIZE -> User cannot resize
    cv::resizeWindow(windowName, 3000, 1500);

    cv::imshow(windowName, imgResult);
    char key = (char)cv::waitKey(0);
    if (key=='s'){
        cv::imwrite(pathModel_ + "ImageStack.jpg", imgResult);
        ROS_INFO("Saved ImageStack: %sImageStack.jpg", pathModel_.c_str());
    } 

    cv::destroyAllWindows();
    ros::shutdown();
    return 0;
}