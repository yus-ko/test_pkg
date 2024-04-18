cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
)

generate_dynamic_reconfigure_options(
  cfg/pcl_fig.cfg
)

catkin_package(
  LIBRARIES potbot_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_pclfig src/pcl_fig.cpp)
target_link_libraries(${PROJECT_NAME}_pclfig ${catkin_LIBRARIES})

add_definitions(-std=c++14)