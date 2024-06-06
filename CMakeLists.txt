cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  geometry_msgs
  nav_msgs
  visualization_msgs
  actionlib_msgs
  potbot_lib
  potbot_msgs
  tf2_ros
  costmap_2d
  base_local_planner
)

generate_dynamic_reconfigure_options(
  cfg/pcl_fig.cfg
  cfg/local_planner.cfg
)

add_action_files(
  FILES
  Navigation.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
  LIBRARIES potbot_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_fig src/pcl_fig.cpp)
add_dependencies(pcl_fig ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_fig ${catkin_LIBRARIES})

add_executable(pso src/pso.cpp)
target_link_libraries(pso ${catkin_LIBRARIES})

add_executable(local_planner src/local_planner.cpp)
add_dependencies(local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_planner ${catkin_LIBRARIES})

add_executable(adv_action_client src/adv_action_client.cpp)
add_executable(adv_action_server src/adv_action_server.cpp)

add_dependencies(adv_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(adv_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(adv_action_client ${catkin_LIBRARIES})
target_link_libraries(adv_action_server ${catkin_LIBRARIES})

add_executable(addr src/addr.cpp)
target_link_libraries(addr ${catkin_LIBRARIES})

add_definitions(-std=c++14)