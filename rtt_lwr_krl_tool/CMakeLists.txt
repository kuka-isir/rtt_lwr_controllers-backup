cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_krl_tool)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    kuka_lwr_fri
    std_msgs
    std_srvs
    krl_msgs
    cmake_modules
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)


target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})


orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros lwr_fri
  )
