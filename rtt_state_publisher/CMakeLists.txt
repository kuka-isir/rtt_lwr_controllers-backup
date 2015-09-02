cmake_minimum_required(VERSION 2.8.3)
project(rtt_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  std_msgs
  sensor_msgs
  urdf
  cmake_modules
)
find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS eigen
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package(
DEPENDS rtt_ros rtt_sensor_msgs
)
