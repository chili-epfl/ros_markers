cmake_minimum_required(VERSION 2.8.3)
project(ros_markers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf cv_bridge image_transport image_geometry)

## System dependencies
find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_search_module(CHILITAGS REQUIRED chilitags)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS tf
  DEPENDS chilitags OpenCV
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${CHILITAGS_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_executable(detect src/detect.cpp)

target_link_libraries(detect
   ${catkin_LIBRARIES}
   ${CHILITAGS_LDFLAGS}
   ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/detect.launch
  config/markers_configuration_sample.yml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

