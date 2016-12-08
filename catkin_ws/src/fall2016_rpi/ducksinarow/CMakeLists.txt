cmake_minimum_required(VERSION 2.8.3)
project(ducksinarow)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  roscpp
  rospy
  message_generation
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime duckietown_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
##add_dependencies(ducksinarow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(ducksinarow_node src/ducksinarow_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ducksinarow_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ducksinarow_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

##add_executable(ducksinarow_node src/ducksinarow_node.cpp)
##target_link_libraries(ducksinarow_node ${catkin_LIBRARIES})
##add_dependencies(ducksinarow_node ${catkin_EXPORTED_TARGETS})

