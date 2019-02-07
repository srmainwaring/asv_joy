cmake_minimum_required(VERSION 2.8.3)
project(asv_joy_gazebo_plugins)

###############################################################################
# Compile as C++11, supported in ROS Kinetic and newer

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############################################################################
# Other dependencies...

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SDL REQUIRED sdl2)

############################################################################### 
# Catkin...

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    JoySDLPlugin
    TeleopTwistJoyPlugin
    LinkJointControllerPlugin
  CATKIN_DEPENDS
    gazebo_ros
  DEPENDS
    Boost
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIRS}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  ${SDL_LIBRARY_DIRS}
  )

############################################################################### 
# Libraries...

############################################################################### 
# Plugins...

# JoySDLPlugin
add_library(JoySDLPlugin
  SHARED
    src/JoySDLPlugin.cc
)

target_link_libraries(JoySDLPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${SDL_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(JoySDLPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_JOY_GAZEBO_PLUGINS_LIST JoySDLPlugin)

# TeleopTwistJoyPlugin
add_library(TeleopTwistJoyPlugin
  SHARED
    src/TeleopTwistJoyPlugin.cc
)

target_link_libraries(TeleopTwistJoyPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(TeleopTwistJoyPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_JOY_GAZEBO_PLUGINS_LIST TeleopTwistJoyPlugin)

# LinkJointControllerPlugin
add_library(LinkJointControllerPlugin
  SHARED
    src/LinkJointControllerPlugin.cc
)

target_link_libraries(LinkJointControllerPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(LinkJointControllerPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_JOY_GAZEBO_PLUGINS_LIST LinkJointControllerPlugin)

############################################################################### 
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

if(CATKIN_ENABLE_TESTING)

  # catkin_add_gtest(UNIT_JoySDLPlugin_TEST src/JoySDLPlugin_TEST.cc)
  # target_link_libraries(UNIT_JoySDLPlugin_TEST ${ASV_JOY_GAZEBO_PLUGINS_LIST})
  # target_compile_options(UNIT_JoySDLPlugin_TEST PRIVATE "-Wno-unknown-pragmas")

endif()

############################################################################### 
# Install

install(TARGETS ${ASV_WAVE_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)
