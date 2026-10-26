cmake_minimum_required(VERSION 3.16)
project(fw_mini_watchdog_test LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp tf tf2_ros tf2_geometry_msgs yhs_can_msgs)
set(FW_MINI_DRIVER_SOURCE "" CACHE PATH "FW-mini-ros/yhs_can_control source directory")
if(NOT EXISTS "${FW_MINI_DRIVER_SOURCE}/src/yhs_can_control.cpp")
  message(FATAL_ERROR "Set FW_MINI_DRIVER_SOURCE to the patched yhs_can_control package")
endif()
add_library(driver_under_test OBJECT "${FW_MINI_DRIVER_SOURCE}/src/yhs_can_control.cpp")
target_compile_definitions(driver_under_test PRIVATE main=yhs_driver_main)
target_include_directories(driver_under_test PRIVATE ${catkin_INCLUDE_DIRS}
  "${FW_MINI_DRIVER_SOURCE}/include/yhs_can_control")
add_executable(watchdog_pipe_test watchdog_pipe_test.cpp $<TARGET_OBJECTS:driver_under_test>)
target_include_directories(watchdog_pipe_test PRIVATE ${catkin_INCLUDE_DIRS}
  "${FW_MINI_DRIVER_SOURCE}/include/yhs_can_control")
target_link_libraries(watchdog_pipe_test PRIVATE ${catkin_LIBRARIES})
# Assertions in this transport test must also run in Release builds.
target_compile_options(watchdog_pipe_test PRIVATE -UNDEBUG)
