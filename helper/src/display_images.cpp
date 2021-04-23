
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>         // in createTrainImages to create model folder

#include <unistd.h>
#include <iostream>
#include <std_msgs/String.h>

#include <time.h>

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


std::vector<cv::Mat> getHorizontalStack(int n, std::string path,  std::string extraPath, std::string name, int id )
{
    std::vector<cv::Mat> matrices;
    int i = 1;
    std::vector<std::string> alpha = {"20", "25", "35", "35", "35", "35", "45", "55", "62"};
    std::vector<std::string> hk = {"0,55","0,65", "0,63", "0,88","1,15", "1,38", "1,09", "1,26", "1,36" };

    while (ros::ok() && i <= n) 
    { 
        cv::Mat img; 
        std::string filename;
        if (extraPath.empty()){
            filename = path + name + "/" + name + std::to_string(i) + "_color.jpg";
        } else {
            filename = path + name + extraPath + "/" + name + std::to_string(i) + "_color.jpg";
        }
                  
        img = cv::imread(filename);
        if (img.empty()) { ROS_ERROR("Could not open or find the image: %s", filename.c_str());}      

        if (i == 1) {
            ROS_INFO("Start loading with image %s", filename.c_str() );
            cv::putText(img, alpha[id] +"deg; " + hk[id] + "m", cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.5, cv::Scalar(255,255,255), 4, cv:: LINE_AA); 
        } else {
            cv::putText(img, std::to_string(i), cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.5, cv::Scalar(255,255,255), 4, cv:: LINE_AA); 
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

    std::string pathModel_, nameSave_, extraFolder_; 
    int numberImages_;
    std::vector<std::string> object_name;

    ros::NodeHandle nh("~");
    nh.param<std::string>("nameSave", nameSave_, "");
    nh.param<std::string>("extraFolder", extraFolder_, "");
    nh.param<std::string>("path", pathModel_, "");                                            // path to package, e.g learn_object/model/
    nh.param<int>("numberOfImages", numberImages_, 3);
    nh.getParam("targets", object_name); 
    ROS_INFO("Node to display images started" );

    // ROS_INFO("size: %zu",object_name.size() );
    std::vector<cv::Mat> matricesV;
    int i = 0;
    while (ros::ok() && i < object_name.size()) 
    {   
        ROS_INFO("Loading image series number %d: %s", i, object_name[i].c_str() );
        cv::Mat imgStack;
        std::vector<cv::Mat> matrices;
        matrices = getHorizontalStack(numberImages_, pathModel_, extraFolder_, object_name[i], i);
        cv::hconcat(matrices, imgStack);
        matricesV.push_back(imgStack);
        i++;
    }
    cv::Mat imgResult;
    cv::vconcat(matricesV, imgResult);
    // // cv::resize(imgResult, imgResult, cv::Size(imgResult.cols/2, imgResult.rows/2));
   
    cv::String windowName = "ImageStack ";     
    cv::namedWindow(windowName, cv::WINDOW_NORMAL );   //cv::WINDOW_AUTOSIZE -> User cannot resize
    cv::resizeWindow(windowName, 3000, 1500);

    cv::imshow(windowName, imgResult);
    char key = (char)cv::waitKey(0);
    if (key=='s'){
        time_t rawtime;
        struct tm * timeinfo;
        char buffer [80];
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (buffer,80,"%Ih%Mmin%p",timeinfo);
        std::string saving;

        if (nameSave_.empty())
        {
            saving = pathModel_ + "imageStacks/" + "ImageStack_"+ buffer+ ".jpg";
        } else {
            saving = pathModel_ + "imageStacks/" + nameSave_ +".jpg";
        }  
        cv::imwrite(saving, imgResult);      
        ROS_INFO("Saved ImageStack: %s", saving.c_str());
    } 

    cv::destroyAllWindows();
    ros::shutdown();
    return 0;
}