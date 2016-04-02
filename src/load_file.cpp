#include "ros/ros.h"
#include <ros/package.h>


#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <algorithm> // for std::copy

int main(int argc, char **argv) {
  
  // Initialize
  ros::init(argc, argv, "node_name");
  ros::NodeHandle nh; 
  
  // Set file name and path
  std::string file = ros::package::getPath("tobor") +"/plan.txt";
  
  // Load file
  std::ifstream is(file.c_str());
  std::istream_iterator<double> start(is), end;
  std::vector<double> data(start, end);
  std::cout << "Read " << data.size() << " numbers" << std::endl;
  
  // Sort data columns into vectors
  int columns = 3;
  std::vector<double> path_x (data.size()/columns);
  std::vector<double> path_y (data.size()/columns);
  std::vector<double> path_a (data.size()/columns);

  for (int i =0 ; i < data.size()/columns ; i++){
      path_x[i] = data[columns*i];
      path_y[i] = data.at(columns*i+1);
      path_a[i] = data.at(columns*i+2);
  }// end for
  
  // print the numbers to stdout
  std::cout << "path_x:\n";
  std::copy(path_x.begin(), path_x.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << std::endl;
  
  std::cout << "path_y:\n";
  std::copy(path_y.begin(), path_y.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << std::endl;
  
  std::cout << "path_a:\n";
  std::copy(path_a.begin(), path_a.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << std::endl;
  
}// end main


