cmake_minimum_required(VERSION 3.16)
project(neupan_cpp_ros1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy roslib sensor_msgs nav_msgs geometry_msgs visualization_msgs
  std_msgs std_srvs diagnostic_msgs tf2 tf2_ros)
find_package(libneupan CONFIG REQUIRED)
catkin_package(CATKIN_DEPENDS
  roscpp rospy roslib sensor_msgs nav_msgs geometry_msgs visualization_msgs
  std_msgs std_srvs diagnostic_msgs tf2 tf2_ros)
add_executable(neupan_node src/neupan_node.cpp)
target_include_directories(neupan_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(neupan_node PRIVATE neupan::neupan ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(neupan_node ${catkin_EXPORTED_TARGETS})
endif()
catkin_install_python(PROGRAMS scripts/smoke_test.py scripts/simulation_test.py scripts/rviz_sim.py
  scripts/command_guard.py scripts/fw_mini_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS neupan_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config models rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# astar_global_node.cpp and test/*.py remain ROS2 references, not build targets.
