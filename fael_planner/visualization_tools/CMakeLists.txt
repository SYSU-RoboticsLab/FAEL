
cmake_minimum_required(VERSION 3.0.2)
project(visualization_tools)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
        tf
        geometry_msgs
        nav_msgs
        ufomap_manager
        message_generation
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
add_message_files(
        FILES
        ExploredVolumeTime.msg
        ExploredVolumeTravedDist.msg
        TravedDistTime.msg
        ExploredVolumeTravedDistTime.msg
        IterationTime.msg
        ViewpointGain.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES visualization_tools
        CATKIN_DEPENDS  rospy tf geometry_msgs roscpp sensor_msgs std_msgs ufomap_manager pcl_ros nav_msgs message_runtime
        #  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
        include
#        /usr/local/include # Location when using 'make system_install'
#  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
#link_directories(
#  /usr/local/lib # Location when using 'make system_install'
#  /usr/lib       # More usual location (e.g. when installing using a package)
#)


add_executable(exploration_data  src/exploration_data.cpp)
target_link_libraries(exploration_data ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#install(TARGETS visualizationTools
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
#install(PROGRAMS scripts/realTimePlot.py
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
#)

