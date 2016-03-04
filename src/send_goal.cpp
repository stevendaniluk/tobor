// Test for using Runner class

#include <ros/ros.h>
#include <runner.h>
#include <cstdlib>

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "class_test");
  
  // Check input arguments
  if (argc != 4 && argc!=5) {
    ROS_INFO("Usage: Input arguments [x, y, theta] or [x, y, theta, frame_id]");
    return 1;
  }// end if
  
  // Create Runner object
  Runner tobor;
  
  // Assign input arguments to goal object
  tobor.setCurrentGoal(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  
  // Assign goal frame, if frame argument is given
  if (argc == 5) {
    tobor.current_goal.target_pose.header.frame_id = argv[4];
  }// end if
  
  // Send a goal
  tobor.sendGoal(tobor.current_goal);
  
  tobor.showPose();
  
  return 0;
}


