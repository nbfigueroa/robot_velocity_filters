cmake_minimum_required(VERSION 2.8.3)
project(online_motion_generator)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall -Wno-deprecated-declarations -Wno-unused-variable -Wno-reorder")



## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mathlib
  sgaussians
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES online_motion_generator
 CATKIN_DEPENDS roscpp rospy std_msgs mathlib sgaussians
)


###########
## Build ##
###########

## Specify locations of header and source files
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(H_FILES ${INCLUDE_DIR}/velocity_filters/CDDynamics_twist_filter.h
			${INCLUDE_DIR}/velocity_filters/CDDynamics_joint_filter.h)

set(S_FILES ${SRC_DIR}/velocity_filters/CDDynamics_twist_filter.cpp
			${SRC_DIR}/velocity_filters/CDDynamics_joint_filter.cpp)


## Declare the C++ library
include_directories(include/velocity_filters ${catkin_INCLUDE_DIRS})
add_library(cddynamics_filter ${H_FILES} ${S_FILES} )
target_link_libraries(cddynamics_filter ${catkin_LIBRARIES})


## Declare C++ executable for twist filter node
add_executable(cddynamics_twist_filter_node src/cddynamics_twist_filter_node.cpp)
target_link_libraries(cddynamics_twist_filter_node cddynamics_filter)

## Declare C++ executable for joint filter node
add_executable(cddynamics_joint_filter_node src/cddynamics_joint_filter_node.cpp)
target_link_libraries(cddynamics_joint_filter_node cddynamics_filter)

### ADD Nodes that use the library
add_dependencies(cddynamics_twist_filter_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

add_dependencies(cddynamics_joint_filter_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})


target_link_libraries(cddynamics_twist_filter_node cddynamics_filter ${catkin_LIBRARIES} )
target_link_libraries(cddynamics_joint_filter_node cddynamics_filter ${catkin_LIBRARIES} )