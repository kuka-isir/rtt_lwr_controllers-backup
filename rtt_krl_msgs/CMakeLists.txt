cmake_minimum_required(VERSION 2.8.3)
project(rtt_krl_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(krl_msgs)
ros_generate_rtt_service_proxies(krl_msgs)

orocos_generate_package(
  DEPENDS krl_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_geometry_msgs
 rtt_actionlib_msgs
 rtt_rosgraph_msgs

)
