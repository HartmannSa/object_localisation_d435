#include <detection_localisation/CamDetectionAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Bool.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<detection_localisation::CamDetectionAction> DetectionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CamDetectionClient
{
private:
  const int STATE_SEARCHING = 0;
  const int STATE_FINISH = 1;
  const int STATE_FOUND_MATCH = 2;
  const int STATE_REFINE = 3;

  const int MAX_TRIAL = 1;

  DetectionClient client_detect;
  detection_localisation::CamDetectionGoal detection_goal;
  detection_localisation::CamDetectionFeedback feedbackPrevious_;

  MoveBaseClient client_move;
  move_base_msgs::MoveBaseGoal move_goal;
  geometry_msgs::PoseArray posearray_;
  int move_goal_number;
  int trial;  
  bool reached_end_of_searchpath;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  std::string target_frame_;
  std::string cam_frame_;

public:    
  CamDetectionClient() : client_detect("detection", true), 
                        client_move("move_base", true),
                        tf2_listener_(tf2_buffer_), 
                        target_frame_("map"),
                        cam_frame_("camera_arm_color_optical_frame") 
  {
    while(!client_detect.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the detection action server to come up");}
    ROS_INFO("detection action server is up");
    while(!client_move.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");}
    ROS_INFO("move_base action server is up");   
  }

  ~CamDetectionClient() {}
  
  void detectObject(std::string path_poses, std::string objekt_name, std::string model_file, std::string learning_data, std::string tracker_settings, std::string detection_settings)
  {  
    move_goal_number = 0;
    trial = 0;
    reached_end_of_searchpath = false;
    loadPoses(path_poses);

    detection_goal.object_name = objekt_name;
    detection_goal.file_3Dmodel = model_file;
    detection_goal.file_learning_data = learning_data;  
    detection_goal.file_tracker_config = tracker_settings;
    detection_goal.file_detection_config = detection_settings;  
    client_detect.sendGoal(detection_goal, 
                    boost::bind(&CamDetectionClient::doneCb, this, _1, _2),
                    boost::bind(&CamDetectionClient::activeCb, this),
                    boost::bind(&CamDetectionClient::feedbackCb, this, _1));     
  }

  void loadPoses(std::string path)
  {
    std::ifstream file;    
    float px, py, pz, qx, qy, qz, qw;
    char c;
    geometry_msgs::Pose p;

    file.open(path.c_str(), std::ifstream::in);   

    if (file.is_open())
    {
      posearray_.header.stamp = ros::Time::now(); 
      posearray_.header.frame_id = "map"; 
      std::string line;    
      while ( getline (file,line) )
      {
        std::istringstream iss(line, std::istringstream::in);
        
        if ( !line.length() || (line[0] == '#') )
            continue;   // skip empty lines or lines starting with # (comments)

        // while ( (iss >> px >> c >> py >> c >> pz >> c >> qx >> c >> qy >> c >> qz >> c >> qw) && (c == ',') )
        while ( (iss >> qw >> c >> qx >> c >> qy >> c >> qz >> c >> px >> c >> py >> c >> pz) && (c == ',') )
        {
          p.position.x = px;
          p.position.y = py;
          p.position.z = pz;
          p.orientation.x = qx;
          p.orientation.y = qy;
          p.orientation.z = qz;
          p.orientation.w = qw;
          posearray_.poses.push_back(p);
        }        
      }
      file.close();
    } else ROS_INFO("Unable to open file %s", path.c_str() ); 
  }

  void setMoveGoal(int goal_number, std::string frame = "map")
  {
    move_goal.target_pose.header.stamp = ros::Time::now(); 
    move_goal.target_pose.header.frame_id = frame;
    // Posearry already initialised by loadPoses()
    move_goal.target_pose.pose = posearray_.poses[goal_number];
  }  

  void calcMoveGoal(const geometry_msgs::PoseStamped pose_in_camKOS)
  {
    geometry_msgs::PoseStamped pose_map;
    geometry_msgs::TransformStamped map_to_cam_transform;

   try {
      map_to_cam_transform = tf2_buffer_.lookupTransform(target_frame_, cam_frame_, ros::Time(0));
      tf2::doTransform(pose_in_camKOS, pose_map, map_to_cam_transform);
      ROS_INFO("Pose in Map (x:%f y:%f z:%f)\n", 
             pose_map.pose.position.x,
             pose_map.pose.position.y,
             pose_map.pose.position.z);
    } catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); 
    }

    // Take x,y,z as new goal
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const detection_localisation::CamDetectionResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("position: %.3f; %.3f; %.3f [m]", result->object_pose.pose.position.x, result->object_pose.pose.position.y, result->object_pose.pose.position.z );
    ROS_INFO("orientation: %.2f; %.2f; %.2f [degree]", result->angles.x, result->angles.y, result->angles.z );
    ROS_INFO("rotation stdev: %f; %f; %f", result->rotx_stdev, result->roty_stdev, result->rotz_stdev );
    ROS_INFO("translation stdev: %f; %f; %f", result->px_stdev, result->py_stdev, result->pz_stdev );

    calcMoveGoal(result->object_pose);

    ros::shutdown();
  }

  void activeCb()
  {
    ROS_INFO("Goal just went active");

    setMoveGoal(move_goal_number);
    // ROS_INFO("Sending first nav-goal with number %i", move_goal_number);   
    ROS_INFO("Sending first nav-goal with number %i: %.2f; %.2f; %.2f", move_goal_number, 
              posearray_.poses[move_goal_number].position.x, 
              posearray_.poses[move_goal_number].position.y, 
              posearray_.poses[move_goal_number].position.z);    
    client_move.sendGoal(move_goal);
  } 

  void feedbackCb(const detection_localisation::CamDetectionFeedbackConstPtr& feedback)
  {
    //  if (feedback->state != feedbackPrevious_.state) {ROS_INFO("Got Feedback with state %i", feedback->state);}
    // ROS_INFO("Got Feedback with state %i", feedback->state);
    // ROS_INFO("Object position:    [%.2f; %.2f; %.2f]", feedback->estimated_pose.pose.position.x, feedback->estimated_pose.pose.position.y, feedback->estimated_pose.pose.position.z );
    // ROS_INFO("Object orientation: [%.2f; %.2f; %.2f; %.2f]", feedback->estimated_pose.pose.orientation.x, feedback->estimated_pose.pose.orientation.y, feedback->estimated_pose.pose.orientation.z, feedback->estimated_pose.pose.orientation.w );
    
    actionlib::SimpleClientGoalState moveState = client_move.getState();          
    switch (feedback->state){
      case 0: //STATE_SEARCHING           
        if (moveState == actionlib::SimpleClientGoalState::SUCCEEDED ||
            moveState == actionlib::SimpleClientGoalState::ABORTED ||
            moveState == actionlib::SimpleClientGoalState::REJECTED ||
            moveState == actionlib::SimpleClientGoalState::RECALLED ||
            moveState == actionlib::SimpleClientGoalState::PREEMPTED) 
        {           
          ROS_INFO("Navigation state: %s", moveState.toString().c_str());
          
          if (moveState == actionlib::SimpleClientGoalState::SUCCEEDED){ 
            move_goal_number++;} 
          else if (moveState == actionlib::SimpleClientGoalState::ABORTED ||
                  moveState == actionlib::SimpleClientGoalState::REJECTED) {
            trial++;
            move_goal_number++;
          }
          
          // Send search pose, if the end of search poses is not reached already 
          // e.g. size returns 4, than is the last goal to send 3 (because we start with goal 0)
          if (move_goal_number < posearray_.poses.size()){    
            setMoveGoal(move_goal_number);
            ROS_INFO("Sending nav-goal with number %i: %.2f; %.2f; %.2f", move_goal_number, posearray_.poses[move_goal_number].position.x, posearray_.poses[move_goal_number].position.y, posearray_.poses[move_goal_number].position.z);    
            client_move.sendGoal(move_goal);
          } else {              
            client_detect.cancelGoal();
            ROS_INFO("End of search path reached. Nav-goal number %i = %zu (number of search poses).", move_goal_number, posearray_.poses.size() );
            ROS_INFO("Cancel detection Goal %s.", detection_goal.object_name.c_str()); 
            move_goal_number = 0; // Continue search at Startposition  
          } 
        }  
        break;
      case 1: //(STATE_FINISH):
        break;
      case 2: //(STATE_FOUND_MATCH):
        ROS_INFO("Navigation state: %s", moveState.toString().c_str());
        // If Navigation is still active,cancel it because an object is detected
        if (moveState == actionlib::SimpleClientGoalState::ACTIVE) {
          client_move.cancelGoal();
          // if (move_goal_number != 0) 
          //   move_goal_number--;
          ROS_INFO("Cancel nav-goal %i", move_goal_number);
        } 
        break;
      case 3: //(STATE_REFINE):
        break;
      default:
        ROS_ERROR("The Feedback state %i is not defined!", feedback->state );
        break;
    }

    feedbackPrevious_.estimated_pose = feedback->estimated_pose;            
    feedbackPrevious_.state = feedback->state;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_client");
  ros::NodeHandle nh("~");
  std::string path, object_name, learning_data, model_file, tracker_settings, detection_settings;
  nh.param<std::string>("path_searchposes", path, "src/object_localisation_d435/detection_localisation/config/searchPoses.config");
  nh.param<std::string>("object_name", object_name, "");
  nh.param<std::string>("file_3Dmodel", model_file, "");
  nh.param<std::string>("learning_data", learning_data, "");
  nh.param<std::string>("tracker_settings", detection_settings, "");
  nh.param<std::string>("detection_settings", detection_settings, "");

  
  CamDetectionClient cam_detection_client;
  cam_detection_client.detectObject(path, object_name, model_file, learning_data, tracker_settings, detection_settings);
  ros::spin();
  return 0;
}