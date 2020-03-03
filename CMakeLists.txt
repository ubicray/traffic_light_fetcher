cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_fetcher)

find_package(catkin REQUIRED
rospy
std_msgs
sensor_msgs
cv_bridge
geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge geometry_msgs
)
