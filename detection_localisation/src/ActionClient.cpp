#include <detection_localisation/CamDetectionAction.h> 
#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <std_msgs/Bool.h>

#include <fstream>

typedef actionlib::SimpleActionClient<detection_localisation::CamDetectionAction> DetectionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""//
//  CLASS CamDetectionClient                                                                                         //
// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""" //
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
  bool manual_ok_for_next_pose_;  // bool moveRobot_;
  bool manual_ok_;
  ros::NodeHandle n_;
  ros::Subscriber sub_;

public:
  
  // ------------------------------------------------------ //
  //  Konstruktor und Destruktor                            //
  // ------------------------------------------------------ //   
  CamDetectionClient() : client_detect("detection", true), 
                        client_move("move_base", true)
  {
    sub_ = n_.subscribe("/manual_ok", 10, &CamDetectionClient::setManualOk, this);
    while(!client_detect.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the detection action server to come up");
    }
    ROS_INFO("detection action server is up");
    while(!client_move.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
     ROS_INFO("move_base action server is up");
  }

  ~CamDetectionClient() {}
  
  // ------------------------------------------------------ //
  //  DetectObject ("Main Funktion")                        //
  // ------------------------------------------------------ //
  void detectObject(std::string objekt_name, std::string learning_data, std::string path_poses, bool debug)
  {  
    if (debug)
    {
      manual_ok_for_next_pose_ = true;
      manual_ok_ = false;
    } else {
      manual_ok_for_next_pose_ = false;
    } 
    move_goal_number = 0;
    trial = 0;
    reached_end_of_searchpath = false;
    loadPoses(path_poses);

    detection_goal.object_name = objekt_name;
    detection_goal.learning_data = learning_data;    
    client_detect.sendGoal(detection_goal, 
                    boost::bind(&CamDetectionClient::doneCb, this, _1, _2),
                    boost::bind(&CamDetectionClient::activeCb, this),
                    boost::bind(&CamDetectionClient::feedbackCb, this, _1));     
  }
  void setManualOk(const std_msgs::Bool::ConstPtr& msg)
  {
    manual_ok_ = msg->data;
    ROS_INFO("%d", msg->data);
  }

  // ------------------------------------------------------ //
  //  Load search poses from config file                           //
  // ------------------------------------------------------ //
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

  // ------------------------------------------------------ //
  //  Set a Goal for move_Base                              //
  // ------------------------------------------------------ //
  void setMoveGoal(int goal_number, std::string frame = "map")
  {
    move_goal.target_pose.header.stamp = ros::Time::now(); 
    move_goal.target_pose.header.frame_id = frame;
    // Posearry already initialised by loadPoses()
    move_goal.target_pose.pose = posearray_.poses[goal_number];
  }  

  // ------------------------------------------------------ //
  //  doneCallback                                          //
  // ------------------------------------------------------ //
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const detection_localisation::CamDetectionResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("position: %.3f; %.3f; %.3f [m]", result->object_pose.pose.position.x, result->object_pose.pose.position.y, result->object_pose.pose.position.z );
    ROS_INFO("orientation: %.2f; %.2f; %.2f [degree]", result->angles.x, result->angles.y, result->angles.z );
    ROS_INFO("rotation stdev: %f; %f; %f", result->rotx_stdev, result->roty_stdev, result->rotz_stdev );
    ROS_INFO("translation stdev: %f; %f; %f", result->px_stdev, result->py_stdev, result->pz_stdev );
    ros::shutdown();
  }

  // ------------------------------------------------------ //
  //  activeCallback                                        //
  // ------------------------------------------------------ //
  void activeCb()
  {
    ROS_INFO("Goal just went active");

    setMoveGoal(move_goal_number);
    // ROS_INFO("Sending first nav-goal with number %i", move_goal_number);   
    ROS_INFO("Sending first nav-goal with number %i: %.2f; %.2f; %.2f", move_goal_number, posearray_.poses[move_goal_number].position.x, posearray_.poses[move_goal_number].position.y, posearray_.poses[move_goal_number].position.z);    
    client_move.sendGoal(move_goal);
  } 
  
  // ------------------------------------------------------ //
  //  feedbackCallback                                      //
  // ------------------------------------------------------ //
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

        // DEBUG
        // if (manual_ok_) 
        // {
        //   ROS_INFO("Navigation state: %s", moveState.toString().c_str());
        //   move_goal_number++;
        //   setMoveGoal(move_goal_number);
        //   ROS_INFO("Sending nav-goal with number %i: %.2f; %.2f; %.2f", move_goal_number, posearray_.poses[move_goal_number].position.x, posearray_.poses[move_goal_number].position.y, posearray_.poses[move_goal_number].position.z);    
        //   client_move.sendGoal(move_goal);
        //   manual_ok_ = false;
        // }   

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

// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""//
//  MAIN                                                                                                             //
// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""" //
int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_client");
  ros::NodeHandle nh("~");
  std::string path;
  std::string object_name;
  std::string learning_data;
  bool debug;
  nh.param<std::string>("object_name", object_name, "Teabox");
  nh.param<std::string>("learning_data", learning_data, "Teabox0_learning_data.bin");
  nh.param<std::string>("path_searchposes", path, "src/object_localisation_d435/detection_localisation/config/searchPoses.config");
  nh.param<bool>("debug", debug, false);

  
  CamDetectionClient cam_detection_client;
  cam_detection_client.detectObject(object_name, learning_data, path, debug);
  ros::spin();
  return 0;
}