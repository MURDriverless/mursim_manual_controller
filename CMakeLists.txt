cmake_minimum_required(VERSION 3.0.2)
project(mursim_controller)

find_package(catkin REQUIRED COMPONENTS
  mur_common
  roscpp
  rospy
  std_msgs
  message_generation
)



catkin_package()


include_directories(${catkin_INCLUDE_DIRS})


add_executable(keyboard mursim_controller/src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})

