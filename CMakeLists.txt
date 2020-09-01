cmake_minimum_required(VERSION 3.0.2)
project(ros_detection_legs)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  ros_pygame_radar_2D
  dynamic_reconfigure
)

catkin_python_setup() 

generate_dynamic_reconfigure_options(
  cfg/HyperParameters.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_detection_legs
   CATKIN_DEPENDS rospy sensor_msgs ros_pygame_radar_2D dynamic_reconfigure
#  DEPENDS system_lib
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
