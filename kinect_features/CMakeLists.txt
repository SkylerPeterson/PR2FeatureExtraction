cmake_minimum_required(VERSION 2.8.3)
project(kinect_features)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.7.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_ros
  pcl_conversions
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  temp.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinect_features
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES})
add_dependencies(cluster_extraction ${catkin_EXPORTED_TARGETS})

add_executable (normal_extraction src/normal_extraction.cpp)
target_link_libraries(normal_extraction ${catkin_LIBRARIES})
add_dependencies(normal_extraction ${catkin_EXPORTED_TARGETS})

add_executable (ConvertToPCLPointCloud src/ConvertToPCLPointCloud.cpp)
target_link_libraries(ConvertToPCLPointCloud ${catkin_LIBRARIES})
add_dependencies(ConvertToPCLPointCloud ${catkin_EXPORTED_TARGETS})

#add_executable (cluster_feature_extraction_subscriber src/cluster_feature_extraction_subscriber.cpp)
#target_link_libraries(cluster_feature_extraction_subscriber ${catkin_LIBRARIES})
#add_dependencies(cluster_feature_extraction_subscriber ${catkin_EXPORTED_TARGETS})

add_executable (cluster_feature_extraction src/cluster_feature_extraction.cpp)
target_link_libraries (cluster_feature_extraction ${PCL_LIBRARIES})

#add_executable (openni_grabber src/openni_grabber.cpp)
#target_link_libraries (openni_grabber ${PCL_LIBRARIES})
#add_dependencies(openni_grabber ${catkin_EXPORTED_TARGETS})
