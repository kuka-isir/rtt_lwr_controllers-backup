cmake_minimum_required(VERSION 2.8.3)
project(rtt_dynamics_analysis)

find_package(catkin REQUIRED COMPONENTS 
    rtt_ros
    rtt_rosclock
    rtt_roscomm
    rtt_rosparam
    diagnostic_msgs 
    geometry_msgs 
    std_msgs 
    kdl_conversions
    cmake_modules
    conman
    rtt_ros_kdl_tools
)

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS CONMAN)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

set(OROCOS_NO_AUTO_LINKING TRUE)

catkin_package(
   DEPENDS eigen rtt 
   CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs
   )
   

include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${Eigen_INCLUDE_DIRS} 
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)


if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
endif()

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)


target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
)

orocos_generate_package(INCLUDE_DIRS include)

