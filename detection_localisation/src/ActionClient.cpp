#include <detection_localisation/CamDetectionAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include "geometry_msgs/PoseStamped.h"
// #include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

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
  bool changed_goal_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  std::string cam_frame_, base_frame_, map_frame_;
  float distance_thres_;
  float distance_base_target_;

  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  geometry_msgs::PoseStamped robotPose_;
  geometry_msgs::PoseStamped poseObject_baseKOS_;

public:    
  CamDetectionClient() : client_detect("detection", true), 
                        client_move("move_base", true),
                        tf2_listener_(tf2_buffer_),
                        map_frame_("map"),
                        base_frame_("base_link"), 
                        cam_frame_("camera_arm_color_optical_frame") 
  {
    while(!client_detect.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the detection action server to come up");}
    ROS_INFO("detection action server is up");
    while(!client_move.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");}
    ROS_INFO("move_base action server is up");   
    sub_ = n_.subscribe("/robot_pose_stamped", 1, &CamDetectionClient::getRobotPose, this);
    pub_ = n_.advertise<std_msgs::String>("/searchPose_reached",1000);
    changed_goal_ = false;
  }

  ~CamDetectionClient() {}
  
  void getRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    robotPose_.pose = msg->pose;
    robotPose_.header.frame_id = msg->header.frame_id;
  }

  void detectObject(std::string path_poses, std::string objekt_name, std::string model_file, std::string learning_data, std::string tracker_settings, std::string detection_settings)
  {  
    move_goal_number = 0;
    trial = 0;
    distance_thres_ = 1.8;
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

  void setMoveGoal(int goal_number)
  {
    move_goal.target_pose.header.stamp = ros::Time::now(); 
    move_goal.target_pose.header.frame_id = map_frame_;
    // Posearry already initialised by loadPoses()
    move_goal.target_pose.pose = posearray_.poses[goal_number];
  }  

  bool calcDistance(const geometry_msgs::PoseStamped pose_in_camKOS)
  {
    geometry_msgs::PoseStamped targetPose_baseKOS_, targetPose_mapKOS_;
    geometry_msgs::TransformStamped transform;
    bool succeded = false;

   try {
      transform = tf2_buffer_.lookupTransform(base_frame_, cam_frame_, ros::Time(0));
      tf2::doTransform(pose_in_camKOS, poseObject_baseKOS_, transform);
      distance_base_target_ = sqrt(pow(poseObject_baseKOS_.pose.position.x,2) + pow(poseObject_baseKOS_.pose.position.y,2) + pow(poseObject_baseKOS_.pose.position.z,2));
      ROS_INFO("Distanz: %f", distance_base_target_);   
      if (abs(poseObject_baseKOS_.pose.position.z) < 0.2)
      {

        if (distance_base_target_ > distance_thres_)  
        {
          float ratio = 0.2;
          targetPose_baseKOS_.header.stamp = ros::Time::now();
          targetPose_baseKOS_.header.frame_id = base_frame_;
          targetPose_baseKOS_.pose.position.x = ratio*poseObject_baseKOS_.pose.position.x;
          targetPose_baseKOS_.pose.position.y = ratio*poseObject_baseKOS_.pose.position.y;
          targetPose_baseKOS_.pose.position.z = 0;

          double angle = tan(poseObject_baseKOS_.pose.position.y/poseObject_baseKOS_.pose.position.x);
          tf2::Quaternion quatern_angle;
          quatern_angle.setRPY(0,0, angle);
          targetPose_baseKOS_.pose.orientation = tf2::toMsg(quatern_angle);        

          geometry_msgs::TransformStamped map_to_base_transform;
          try {
            map_to_base_transform = tf2_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
            tf2::doTransform(targetPose_baseKOS_, targetPose_mapKOS_, map_to_base_transform);
            move_goal.target_pose = targetPose_mapKOS_;
            move_goal.target_pose.header.stamp = ros::Time::now();
            succeded = true;

            ROS_INFO("Sending Pose in Map (x:%f y:%f z:%f  mit  qx:%f qy:%f qz:%f qw:%f)", 
            move_goal.target_pose.pose.position.x,
            move_goal.target_pose.pose.position.y,
            move_goal.target_pose.pose.position.z,
            move_goal.target_pose.pose.orientation.x,
            move_goal.target_pose.pose.orientation.y,
            move_goal.target_pose.pose.orientation.z,
            move_goal.target_pose.pose.orientation.w);
            client_move.sendGoal(move_goal);
            changed_goal_ = true;

            std_msgs::String Away;
            Away.data = "away";
            ROS_INFO("Sending away");
            pub_.publish(Away);

          } catch (tf2::TransformException &ex) {
            ROS_WARN("Failure %s\n", ex.what()); 
          }
        } else {
          std_msgs::String Reached;
          Reached.data = "reached";
          ROS_INFO("Sending reached");
          pub_.publish(Reached);
          if (client_move.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            client_move.cancelGoal();
          }
        }
      } else {
        ROS_INFO("Rejected object pose. Z = %f is not on the ground.", poseObject_baseKOS_.pose.position.z );
      }
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what()); 
    }
    return succeded;
  }

  bool setNewMoveGoal(const geometry_msgs::PoseStamped pose_in_camKOS)
  {
    geometry_msgs::PoseStamped pose_map;
    geometry_msgs::TransformStamped map_to_cam_transform;
    bool succeded = false;

   try {
      map_to_cam_transform = tf2_buffer_.lookupTransform(map_frame_, cam_frame_, ros::Time(0));
      tf2::doTransform(pose_in_camKOS, pose_map, map_to_cam_transform);
      ROS_INFO("Receiving Pose in Map (x:%f y:%f z:%f)", //  mit  qx:%f qy:%f qz:%f qw:%f)", 
             pose_map.pose.position.x,
             pose_map.pose.position.y,
             pose_map.pose.position.z);
            //  pose_map.pose.orientation.x,
            //  pose_map.pose.orientation.y,
            //  pose_map.pose.orientation.z,
            //  pose_map.pose.orientation.w);  
      geometry_msgs::Pose targetApproach;
      geometry_msgs::Pose robotPose = robotPose_.pose;
      float ratio = 0.2;
      targetApproach.position.x = robotPose.position.x + ratio * (pose_map.pose.position.x - robotPose.position.x);
      targetApproach.position.y = robotPose.position.y + ratio * (pose_map.pose.position.y - robotPose.position.y);
      targetApproach.position.z = 0;
      
      tf2::Quaternion quatern_angle, quatern_base, quatern_result;
      tf2::convert(robotPose_.pose.orientation, quatern_base);
      double angle = tan(pose_map.pose.position.y/pose_map.pose.position.x);
      quatern_angle.setRPY(0,0, angle);
      quatern_result = quatern_angle*quatern_base;
      quatern_result.normalize();
      targetApproach.orientation = tf2::toMsg(quatern_result);
     
      move_goal.target_pose.header.stamp = ros::Time::now();
      move_goal.target_pose.header.frame_id = map_frame_;
      move_goal.target_pose.pose.position = targetApproach.position;
      move_goal.target_pose.pose.orientation = targetApproach.orientation;

      ROS_INFO("Sending Pose in Map (x:%f y:%f z:%f  mit  qx:%f qy:%f qz:%f qw:%f)", 
        move_goal.target_pose.pose.position.x,
        move_goal.target_pose.pose.position.y,
        move_goal.target_pose.pose.position.z,
        move_goal.target_pose.pose.orientation.x,
        move_goal.target_pose.pose.orientation.y,
        move_goal.target_pose.pose.orientation.z,
        move_goal.target_pose.pose.orientation.w);  
      succeded = true;

    } catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what()); 
    }
    return succeded;
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const detection_localisation::CamDetectionResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("position: %.3f; %.3f; %.3f [m]", result->object_pose.pose.position.x, result->object_pose.pose.position.y, result->object_pose.pose.position.z );
    ROS_INFO("orientation: %.2f; %.2f; %.2f [degree]", result->angles.x, result->angles.y, result->angles.z );
    ROS_INFO("rotation stdev: %f; %f; %f", result->rotx_stdev, result->roty_stdev, result->rotz_stdev );
    ROS_INFO("translation stdev: %f; %f; %f", result->px_stdev, result->py_stdev, result->pz_stdev );

    // calcMoveGoal(result->object_pose);

    ros::shutdown();
  }

  void activeCb()
  {
    ROS_INFO("Goal just went active");

    setMoveGoal(move_goal_number);
    // ROS_INFO("Sending first nav-goal with number %i", move_goal_number);   
    ROS_INFO("Sending first nav-goal with number %i: %.2f; %.2f; %.2f", move_goal_number+1, 
              posearray_.poses[move_goal_number].position.x, 
              posearray_.poses[move_goal_number].position.y, 
              posearray_.poses[move_goal_number].position.z);    
    client_move.sendGoal(move_goal);
    changed_goal_ = false;
  } 

  void feedbackCb(const detection_localisation::CamDetectionFeedbackConstPtr& feedback)
  {
    //  if (feedback->state != feedbackPrevious_.state) {ROS_INFO("Got Feedback with state %i", feedback->state);}
    ROS_INFO("FEEDBACK state %i", feedback->state);
    // ROS_INFO("Object position:    [%.2f; %.2f; %.2f]", feedback->estimated_pose.pose.position.x, feedback->estimated_pose.pose.position.y, feedback->estimated_pose.pose.position.z );
    // ROS_INFO("Object orientation: [%.2f; %.2f; %.2f; %.2f]", feedback->estimated_pose.pose.orientation.x, feedback->estimated_pose.pose.orientation.y, feedback->estimated_pose.pose.orientation.z, feedback->estimated_pose.pose.orientation.w );
    
    actionlib::SimpleClientGoalState moveState = client_move.getState();          
    switch (feedback->state){
      case 0: //STATE_SEARCHING 
      {          
        if (moveState == actionlib::SimpleClientGoalState::SUCCEEDED ||
            moveState == actionlib::SimpleClientGoalState::ABORTED ||
            moveState == actionlib::SimpleClientGoalState::REJECTED ||
            moveState == actionlib::SimpleClientGoalState::RECALLED ||
            moveState == actionlib::SimpleClientGoalState::PREEMPTED) 
        {           
          ROS_INFO("Navigation state: %s", moveState.toString().c_str());
          
          if (moveState == actionlib::SimpleClientGoalState::SUCCEEDED && !changed_goal_){ 
            move_goal_number++;} 
          else if (!changed_goal_ && (moveState == actionlib::SimpleClientGoalState::ABORTED ||
                  moveState == actionlib::SimpleClientGoalState::REJECTED)) {
            trial++;
            move_goal_number++;
          }
          
          // Send search pose, if the end of search poses is not reached already 
          // e.g. size returns 4, than is the last goal to send 3 (because we start with goal 0)
          if (move_goal_number < posearray_.poses.size()){    
            setMoveGoal(move_goal_number);
            ROS_INFO("Sending nav-goal with number %i: %.2f; %.2f; %.2f", move_goal_number+1, posearray_.poses[move_goal_number].position.x, posearray_.poses[move_goal_number].position.y, posearray_.poses[move_goal_number].position.z);    
            client_move.sendGoal(move_goal);
            changed_goal_ = false;
          } else {              
            client_detect.cancelGoal();
            ROS_INFO("End of search path reached. Nav-goal number %i = %zu (number of search poses).", move_goal_number+1, posearray_.poses.size() );
            ROS_INFO("Cancel detection Goal %s.", detection_goal.object_name.c_str()); 
            move_goal_number = 0; // Continue next search at Startposition  
          } 
        }  
        break;
      }
      case 1: //(STATE_FINISH):
        break;
      case 2: //(STATE_FOUND_MATCH):
        ROS_INFO("Navigation state: %s", moveState.toString().c_str());
        // If Navigation is still active,cancel it because an object is detected
        // if (moveState == actionlib::SimpleClientGoalState::ACTIVE) {
        //   client_move.cancelGoal();
        //   ROS_INFO("Cancel nav-goal %i", move_goal_number+1);
        // }

        if (calcDistance(feedback->estimated_pose)) {
          // if (distance_base_target_ > distance_thres_)
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