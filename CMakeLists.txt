cmake_minimum_required(VERSION 2.8.3)
project(csl_detection)

#set(PCL_DIR "/usr/share/pcl-1.7/PCLConfig.cmake")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  #OpenCV
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 EXACT REQUIRED)

find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")

#find_package(Eigen REQUIRED)

set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_VERBOSE_MAKEFILE ON)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  CATKIN_DEPENDS std_msgs geometry_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${freenect2_INCLUDE_DIRS}
  include
#  /usr/include/pcl-1.7/
  ${PCL_INCLUDE_DIRS}
)

#include_directories(${PCL_INCLUDE_DIRS})
#include_directories(${Eigen_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




set(PCL_BUILD_TYPE Release)
## Declare a C++ library
# add_library(csl_detection
#   src/${PROJECT_NAME}/csl_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(csl_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(csl_detection_node src/csl_detection_node.cpp)
#add_executable(csl_tracker_node src/csl_tracker.cpp)
#add_executable(kinect2_tracking_node src/kinect2_tracking_node.cpp)
#add_executable(kinect2_read_write_node src/kinect2_read_write_node.cpp)
add_executable(kinect2_csl_tracker src/kinect2_freenect_tracker.cpp)
add_executable(dian_csl_tracker src/dian_kinect2_tracker.cpp)
add_executable(openni2_grabber_test src/openni2_test.cpp)
add_executable(pcd_cloud_viewer src/pcd_cloud_viewer.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(csl_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(csl_detection_node
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

#target_link_libraries(csl_tracker_node
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

#target_link_libraries(kinect2_tracking_node
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${freenect2_LIBRARY}
##  -lfreenect2
#)

#string(REPLACE -lpcl_recognition    /usr/lib/libpcl_recognition.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_filters    /usr/lib/libpcl_filters.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_common    /usr/lib/libpcl_common.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_apps    /usr/lib/libpcl_apps.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_surface    /usr/lib/libpcl_surface.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_kdtree    /usr/lib/libpcl_kdtree.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_tracking    /usr/lib/libpcl_tracking.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_visualization    /usr/lib/libpcl_visualization.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_io    /usr/lib/libpcl_io.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_outofcore    /usr/lib/libpcl_outofcore.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_people    /usr/lib/libpcl_people.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_keypoints    /usr/lib/libpcl_keypoints.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_features    /usr/lib/libpcl_features.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_search    /usr/lib/libpcl_search.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_sample_consensus    /usr/lib/libpcl_sample_consensus.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_octree    /usr/lib/libpcl_octree.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_io_ply    /usr/lib/libpcl_io_ply.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_segmentation    /usr/lib/libpcl_segmentation.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")
#string(REPLACE -lpcl_registration    /usr/lib/libpcl_registration.so.1.7.2     PCL_LIBRARIES    "${PCL_LIBRARIES}")


target_link_libraries(openni2_grabber_test
  ${catkin_LIBRARIES}
  #${Eigen_LIBRARIES}
# /usr/lib/libpcl_recognition.so.1.7.2
# /usr/lib/libpcl_filters.so.1.7.2
# /usr/lib/libpcl_common.so.1.7.2
# /usr/lib/libpcl_apps.so.1.7.2
# /usr/lib/libpcl_surface.so.1.7.2
# /usr/lib/libpcl_kdtree.so.1.7.2
# /usr/lib/libpcl_tracking.so.1.7.2
# /usr/lib/libpcl_visualization.so.1.7.2
# /usr/lib/libpcl_io.so.1.7.2
# /usr/lib/libpcl_outofcore.so.1.7.2
# /usr/lib/libpcl_people.so.1.7.2
# /usr/lib/libpcl_keypoints.so.1.7.2
# /usr/lib/libpcl_features.so.1.7.2
# /usr/lib/libpcl_search.so.1.7.2
# /usr/lib/libpcl_sample_consensus.so.1.7.2
# /usr/lib/libpcl_octree.so.1.7.2
# /usr/lib/libpcl_io_ply.so.1.7.2
# /usr/lib/libpcl_segmentation.so.1.7.2
# /usr/lib/libpcl_registration.so.1.7.2
  ${PCL_LIBRARIES}
  ${openni2_LIBRARIES}
  #${freenect2_LIBRARY}
#  -lfreenect2
)


target_link_libraries(kinect2_csl_tracker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
#  ${freenect2_LIBRARY}
#  -lfreenect2
)

target_link_libraries(pcd_cloud_viewer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
#  ${freenect2_LIBRARY}
#  -lfreenect2
)


target_link_libraries(dian_csl_tracker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
#  ${freenect2_LIBRARY}
#  -lfreenect2
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS csl_detection csl_detection_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_csl_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
