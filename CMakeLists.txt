cmake_minimum_required(VERSION 2.8.3)
project(roverbot)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf laser_geometry)

add_message_files(
  DIRECTORY roverbot_comunication/std_msgs/msg
  FILES
  Obstacles.msg
  Point.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS roverbot_comunication/std_msgs/include
  CATKIN_DEPENDS message_runtime)

install(DIRECTORY roverbot_comunication/std_msgs/include/std_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(talkerLaser roverbot_comunication/src/talkerLaser.cpp)
target_link_libraries(talkerLaser ${catkin_LIBRARIES})

add_executable(listenerLaser roverbot_comunication/src/listenerLaser.cpp)
target_link_libraries(listenerLaser ${catkin_LIBRARIES})


