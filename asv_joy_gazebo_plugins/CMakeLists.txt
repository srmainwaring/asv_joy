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
    SDLJoyPlugin
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

# SDLJoyPlugin
add_library(SDLJoyPlugin
  SHARED
    src/SDLJoyPlugin.cc
)

target_link_libraries(SDLJoyPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${SDL_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(SDLJoyPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_JOY_GAZEBO_PLUGINS_LIST SDLJoyPlugin)

# JoyToForceTorquePlugin
add_library(JoyToForceTorquePlugin
  SHARED
    src/JoyToForceTorquePlugin.cc
)

target_link_libraries(JoyToForceTorquePlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(JoyToForceTorquePlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_JOY_GAZEBO_PLUGINS_LIST JoyToForceTorquePlugin)

############################################################################### 
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

if(CATKIN_ENABLE_TESTING)

  # catkin_add_gtest(UNIT_SDLJoyPlugin_TEST src/SDLJoyPlugin_TEST.cc)
  # target_link_libraries(UNIT_SDLJoyPlugin_TEST ${ASV_JOY_GAZEBO_PLUGINS_LIST})
  # target_compile_options(UNIT_SDLJoyPlugin_TEST PRIVATE "-Wno-unknown-pragmas")

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
