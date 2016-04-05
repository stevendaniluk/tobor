// Tobor test script for going to goal point, and return

#include <ros/ros.h>
#include <runner.h>
#include <cstdlib>

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "tobor_test");
  
  // Check input arguments
  if (argc != 4 && argc!=5) {
    ROS_INFO("Usage: Input arguments [x, y, theta] or [x, y, theta, frame_id]");
    return 1;
  }// end if
  
  // Create Runner object
  Runner Tobor;
  
  // Display current pose, and set start pose
  Tobor.setStartPose();
  
  // Assign input arguments to goal object
  Tobor.setCurrentGoal(atof(argv[1]), atof(argv[2]), atof(argv[3]), 1.0);
  if (argc == 5) {
    Tobor.current_goal.target_pose.header.frame_id = argv[4];
  }// end if
  
  // Send the goal
  Tobor.sendGoal(Tobor.current_goal);
  
  ros::Duration(1.0).sleep();
  
  // If successful, return to start
  if (Tobor.nav_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Returning to start position.");
    Tobor.setCurrentGoal(Tobor.start_pose.pose.pose.position.x, 
                         Tobor.start_pose.pose.pose.position.y, 
                         Tobor.start_pose.pose.pose.orientation.z, 1.0);
    Tobor.sendGoal(Tobor.current_goal);
  }else {
    ROS_INFO("Did not Succeed.");
  }// end if (goal)
  

  return 0;
}
