// Tobor script for performing a full run

// Loads in text file with goal coordinates and moves to each point

#include <ros/ros.h>
#include <ros/package.h>
#include <runner.h>

#include <cstdlib>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <cmath>

//---------------------------------------------------------------------------

void load_file (std::vector<double> &path_x, 
                std::vector<double> &path_y, 
                std::vector<double> &path_a) {
  
  // Set file name and path
  std::string file = ros::package::getPath("tobor") +"/path.txt";
  
  // Load file
  std::ifstream is(file.c_str());
  std::istream_iterator<double> start(is), end;
  std::vector<double> data(start, end);
  
  // Resize vectors appropriately
  int columns = 3;
  path_x.resize(data.size()/columns);
  path_y.resize(data.size()/columns);
  path_a.resize(data.size()/columns);
  
  // Sort data columns into vectors
  for (int i =0 ; i < data.size()/columns ; i++){
      path_x[i] = data[columns*i];
      path_y[i] = data.at(columns*i+1);
      path_a[i] = data.at(columns*i+2);
  }// end for

}// end load_file

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "full_run");
  
  // Create Runner object
  Runner Tobor;
  
  // Get current pose, and set start pose
  Tobor.getPose();
  Tobor.setStartPose();
  
  // Load path file
  std::vector<double> path_x;
  std::vector<double> path_y;
  std::vector<double> path_a;
  load_file(path_x, path_y, path_a);
  
  // Useful parameters
  int num_goals = path_x.size();  // Number of goals
  double goal_tol = 0.75;  // Goal tolerance, within which next goal is sent [m]
  double x_diff;
  double y_diff;
  
  // Check inputs for goal_tol value
  if (argc == 2) {
    goal_tol = atof(argv[1]);
  }else if (argc > 2) {
    ROS_INFO("Usage: Input arguments [goal_tol]");
    return 1;
  }// end if
  
  ROS_INFO("Beginning navigation.");
  
  // Loop through each goal
  for (int i =0 ; i < (num_goals); i++) {
    
    // Set the next goal, and send it
    Tobor.setCurrentGoal(path_x[i], path_y[i], path_a[i]);
    Tobor.current_goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending goal %d.", i+1);
    Tobor.sendTempGoal(Tobor.current_goal);
    
    // Keep checking navigation state until succeeded
    ros::Rate loop_rate(10);
    // TODO change loop condition
    while (ros::ok()) {
      
      // Update state
      Tobor.updateNavState();
      
      // If navigation has failed, execute recovery behaviour
      // TODO add recovery behaviour
      if(Tobor.nav_state == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("Navigation failed. Aborting.");
        return 1;
      }// end if
      
      // Check current pose
      Tobor.getPose();
      x_diff = std::abs(Tobor.pose.pose.pose.position.x - 
                        Tobor.current_goal.target_pose.pose.position.x);
      y_diff = std::abs(Tobor.pose.pose.pose.position.y - 
                        Tobor.current_goal.target_pose.pose.position.y);
      
      // Check if goal criteria has been satisfied
      // If this is the last goal point, only break when SUCCEEDED
      if ((x_diff < goal_tol) && (y_diff < goal_tol) && i < (num_goals - 1)) {
        ROS_INFO("Position within goal tolerance.");
        break;
      }// end if
      
      if(Tobor.nav_state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Goal reached successfully.");
        break;
      }// end if
      
      loop_rate.sleep();

    }// end while
  }// end for
  

  return 0;
}
