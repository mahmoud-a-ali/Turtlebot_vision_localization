cmake_minimum_required(VERSION 2.8.3)
project(turtle_ekf)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## roscpp has to be in find_package. Possibly others. Just add lines.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)

ADD_REQUIRED_DEPENDENCY("roboptim-core")
ADD_REQUIRED_DEPENDENCY("roboptim-trajectory")

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
   Pose2DWithCovariance.msg
#   Message2.msg
 )

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
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs  # Or other packages containing msgs
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
   CATKIN_DEPENDS message_runtime 
#  INCLUDE_DIRS include
#  LIBRARIES move_joint
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## In our labs, you normally don't need to touch that.
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIR} )
#include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wno-format-extra-args")

## Declare a cpp library
# add_library(move_joint
#   src/${PROJECT_NAME}/move_joint.cpp
# )

## Declare a cpp executable
#add_executable(measurement_node src/measurement_node.cpp)
add_executable(ekf_node src/ekf_node.cpp)
add_executable(image_rectifier src/image_rectifier.cpp)
add_executable(image_tiles src/image_tiles.cpp)
add_executable(interactive_color_segment src/interactive_color_segment.cpp)
add_executable(odom_interface src/odom_interface.cpp)
add_executable(ekf_one_measurement_node src/ekf_one_measurement_node.cpp)
add_executable(blob_detector src/blob_detector.cpp)
add_executable(blob_detector_v2 src/blob_detector_v2.cpp)
add_executable(absolute_loc_node src/absolute_loc_node.cpp)
add_executable(optimization src/optimization.cpp)
add_executable(blob_dist_interface src/blob_dist_interface.cpp)
#add_executable(controller src/controller.cpp)
add_executable(circle_visualizer src/circle_visualizer.cpp)
add_executable(beacon_visualizer src/beacon_visualizer.cpp)
add_executable(path_visualizer src/path_visualizer.cpp)
	       



## A familiar example: add_executable(move_joint_node src/move_joint_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(move_joint_node move_joint_generate_messages_cpp)

## Specify libraries to link a library or executable target against

## In our labs, you normally don't need to touch that.
#target_link_libraries(measurement_node           ${catkin_LIBRARIES})
target_link_libraries(ekf_node                   ${catkin_LIBRARIES})
#target_link_libraries(interactive_color_segment  ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(interactive_color_segment  ${catkin_LIBRARIES})
target_link_libraries(odom_interface               ${catkin_LIBRARIES})
target_link_libraries(ekf_one_measurement_node   ${catkin_LIBRARIES})
target_link_libraries(blob_detector              ${catkin_LIBRARIES})
target_link_libraries(blob_detector_v2              ${catkin_LIBRARIES})
target_link_libraries(absolute_loc_node              ${catkin_LIBRARIES})
target_link_libraries(optimization                   ${catkin_LIBRARIES})
target_link_libraries(image_rectifier                ${catkin_LIBRARIES})
target_link_libraries(image_tiles                    ${catkin_LIBRARIES})
target_link_libraries(blob_dist_interface            ${catkin_LIBRARIES})
#target_link_libraries(controller                     ${catkin_LIBRARIES})
target_link_libraries(circle_visualizer              ${catkin_LIBRARIES})
target_link_libraries(beacon_visualizer              ${catkin_LIBRARIES})
target_link_libraries(path_visualizer                ${catkin_LIBRARIES})

PKG_CONFIG_USE_DEPENDENCY(optimization roboptim-core)



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
# install(TARGETS move_joint move_joint_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_move_joint.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



