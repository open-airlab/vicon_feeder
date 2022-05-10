cmake_minimum_required(VERSION 2.8.3)
project(vicon_feeder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # ROS == Indigo.
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  # ROS > Indigo.
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(vicon_feed 
  src/feed_vicon_to_px4.cpp
)
target_link_libraries(vicon_feed
   ${catkin_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS
#   data_collector
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
