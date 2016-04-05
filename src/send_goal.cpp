// Tobor test script for going to goal point, and return

#include <ros/ros.h>
#include <runner.h>
#include <cstdlib>

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "tobor_test");
  
  // Check input arguments
  if (argc != 4 && argc!=5) {
    ROS_INFO("Usage: Input arguments [x, y, theta] or [x, y, theta, dock]");
    return 1;
  }// end if
  
  // Create Runner object
  Runner Tobor;
  
  // Display current pose, and set start pose
  Tobor.setStartPose();
  
  // Assign input arguments to goal object
  Tobor.setCurrentGoal(atof(argv[1]), atof(argv[2]), atof(argv[3]), 1.0);
  
  // Send the goal
  Tobor.sendGoal(Tobor.current_goal);
  
  ros::Duration(1.0).sleep();
  
  // If successful, start docking
  if (Tobor.nav_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    // If docking requested, begin
    if (argc == 5 && (std::strcmp(argv[4], "dock") || std::strcmp(argv[4], "Dock"))) {
      ROS_INFO("Beginning docking.");
      Tobor.dock();
    }// end if
  }else {
    ROS_INFO("Did not succeed, will not dock.");
  }// end if (goal)

  return 0;
}
