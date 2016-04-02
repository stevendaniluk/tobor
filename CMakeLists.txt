cmake_minimum_required(VERSION 2.8.3)
project(tobor)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  roslib
)

# Declare the catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES runners_simulation
#  CATKIN_DEPENDS navigation runners_navigation stage_ros
#  DEPENDS system_lib
)

#set(SOMETHING_HEADER_PATH "../something/include/")

## Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(send_goal src/send_goal.cpp)
add_executable(go_and_return src/go_and_return.cpp)
add_executable(full_run src/full_run.cpp)

# Specify link libraries
target_link_libraries(send_goal ${catkin_LIBRARIES} tobor)
target_link_libraries(go_and_return ${catkin_LIBRARIES} tobor)
target_link_libraries(full_run ${catkin_LIBRARIES} tobor)

# Declare a C++ library
add_library(tobor src/runner/runner.cpp)
# Specify link libraries
target_link_libraries(tobor ${catkin_LIBRARIES})

