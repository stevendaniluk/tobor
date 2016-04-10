// Tobor script for performing a full run

// Loads in text file with goal coordinates and moves to each point

#include <ros/ros.h>
#include <ros/package.h>
#include <runner.h>
#include <geometry_msgs/Twist.h>

#include <cstdlib>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <cmath>

//---------------------------------------------------------------------------

void load_file (std::string filename,
                std::vector<double> &path_x, 
                std::vector<double> &path_y, 
                std::vector<double> &path_z,
                std::vector<double> &path_w) {
  
  // Set file name and path
  std::string full_file = ros::package::getPath("tobor") + "/" + filename;
  
  // Load file
  std::ifstream is(full_file.c_str());
  std::istream_iterator<double> start(is), end;
  std::vector<double> data(start, end);
  
  // Resize vectors appropriately
  int columns = 4;
  path_x.resize(data.size()/columns);
  path_y.resize(data.size()/columns);
  path_z.resize(data.size()/columns);
  path_w.resize(data.size()/columns);
  
  // Sort data columns into vectors
  for (int i =0 ; i < data.size()/columns ; i++){
      path_x[i] = data[columns*i];
      path_y[i] = data.at(columns*i+1);
      path_z[i] = data.at(columns*i+2);
      path_w[i] = data.at(columns*i+3);
  }// end for

}// end load_file

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "full_run");
  
  // Create nodehandle and publisher for sending cmd_vel commands
  ros::NodeHandle nh_tobor;
  ros::Publisher cmd_vel_pub = nh_tobor.advertise<geometry_msgs::Twist>
                            ("/navigation_velocity_smoother/raw_cmd_vel", 1000);
  
  // Create Runner object
  Runner Tobor;
  
  // Get current pose, and set start pose
  Tobor.update();
  Tobor.setStartPose();
  
  // Load path file
  std::vector<double> path_x, path_y, path_z, path_w; 
  std::string path_file = "path.txt";
  load_file(path_file, path_x, path_y, path_z, path_w);
  
  // Useful variables
  ros::Rate loop_rate(10);
  int num_goals = path_x.size();  // Number of goals
  double x_diff;
  double y_diff;
  ros::Time time;
  geometry_msgs::Twist back_up;
  int failed_attempts = 0;
  int goal = 0; // Current goal number
  
  // Tuning parameters
  double back_up_dist = 1.0;  // Recovery backup distance
  back_up.linear.x=-0.25;     // Recovery backup speed
  double goal_tol = 1.50;     // Goal tolerance, within which next goal is sent [m]
  
  
  //---------------------------------
  
  
  // Check input arguments for starting goal number
  if (argc == 2) {
    goal = atoi(argv[1])-1;
  }// end if
  
  ROS_INFO("Beginning navigation.");
  
  // Loop through each goal
  for (goal; goal < (num_goals); goal++) {
    
    // Set the next goal, and send it
    Tobor.setCurrentGoal(path_x[goal], path_y[goal], path_z[goal], path_w[goal]);
    Tobor.current_goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending goal %d.", goal+1);
    Tobor.sendTempGoal(Tobor.current_goal);
    
    // Keep checking navigation state until within tolerance or succeeded
    while (ros::ok()) {
      
      // Update state
      Tobor.update();
      Tobor.updateNavState();
      
      // If navigation has failed, execute recovery behaviour
      if(Tobor.nav_state == actionlib::SimpleClientGoalState::ABORTED){
        
        ROS_INFO("Navigation failed.");
        
        // Increment failures
        failed_attempts++;
        
        // Failure recovery modes
        if (failed_attempts == 1) {
          // Wait 3 seconds
          ros::Duration(3.0).sleep();
        }else if (failed_attempts == 2) {
          // Send commands to backup
          ROS_INFO("Backing up %.2f m", back_up_dist);
          time = ros::Time::now();
          while(ros::ok()) {
            if (ros::Time::now() > (time+ros::Duration(back_up_dist/std::abs(back_up.linear.x)))) {
              break;
            }// end if
            cmd_vel_pub.publish(back_up);
            loop_rate.sleep();
          }// end while
        }else {
          // Wait 1 second
          ros::Duration(1.0).sleep();
          // Send commands to backup half back_up_dist
          ROS_INFO("Waiting, then backing up %.2f m", back_up_dist/2);
          time = ros::Time::now();
          while(ros::ok()) {
            if (ros::Time::now() > 
                (time+ros::Duration(0.5*back_up_dist/std::abs(back_up.linear.x)))) {
              break;
            }// end if
            cmd_vel_pub.publish(back_up);
            loop_rate.sleep();
          }// end while
        }// end failed_attempts if
        
        // Move goal index back one to retry
        goal--;
        
        break;
      }// end ABORTED if
      
      // Check current pose
      x_diff = std::abs(Tobor.pose.pose.pose.position.x - 
                        Tobor.current_goal.target_pose.pose.position.x);
      y_diff = std::abs(Tobor.pose.pose.pose.position.y - 
                        Tobor.current_goal.target_pose.pose.position.y);
      
      // If we are within the last 3 goals (Barfoot's office), lower tolerance
      if (goal >= num_goals - 4){
        goal_tol = 0.2;
      }// end if
      
      // Check if goal criteria has been satisfied
      // If this is the last goal point, only break when SUCCEEDED
      if ((x_diff < goal_tol) && (y_diff < goal_tol) && goal < (num_goals - 1)) {
        ROS_INFO("Position within goal tolerance.");
        // Reset failed attempts
        failed_attempts = 0;
        break;
      }else if(Tobor.nav_state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Goal reached successfully.");
        break;
      }// end if
      
      // Wait before next loop
      loop_rate.sleep();

    }// end while
  }// end for
  
  // Ask if it should dock
  std::cout << "Do you want to dock? ";
  std::string response;
  std::cin >> response;
  if(response == "Yes" || response == "yes") {
    // Begin docking
    Tobor.dock();
  }// end docking if
  
  return 0;
}
