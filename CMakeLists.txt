cmake_minimum_required(VERSION 2.8.3)
project(learning_egraphs)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    smpl
    sbpl_collision_checking
    sbpl_kdl_robot_model
    sbpl_pr2_robot_model
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(sbpl REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES learning_egraphs
  CATKIN_DEPENDS smpl sbpl_collision_checking sbpl_kdl_robot_model
                sbpl_pr2_robot_model
  DEPENDS sbpl
)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${sbpl_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/learning_egraphs.cpp
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gridworld src/gridworld.cpp src/utils.cpp)
target_link_libraries(gridworld
   ${catkin_LIBRARIES}
   ${sbpl_LIBRARIES}
 )

add_executable(callPlanner src/call_planner.cpp)
target_link_libraries(callPlanner
   ${catkin_LIBRARIES}
   ${sbpl_LIBRARIES}
 )

