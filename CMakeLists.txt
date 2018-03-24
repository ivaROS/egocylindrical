cmake_minimum_required(VERSION 2.8.3)
project(egocylindrical)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  message_generation
)


find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)

if(VECT_DETAILS)
    set(SHOW_VECTORIZE_INFO "-fopt-info-vec-all")
else()
     set(SHOW_VECTORIZE_INFO "")
endif()

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

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  EgoCylinderPoints.msg
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

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES egocylindrical
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport roscpp rospy sensor_msgs std_msgs tf tf2 tf_msgs tf_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/egocylindrical.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# This is a separate library partly to reduce compile times, but mostly because it needs to be compiled with certain 'fast-math' optimization options
add_library(generate_range_image src/generate_range_image.cpp)
target_link_libraries(generate_range_image
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(generate_range_image PRIVATE -g -std=c++11 -Wall -ftree-vectorize ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
  #${OpenMP_CXX_FLAGS}
  )

  
add_library(transform_points src/transform_points.cpp)
target_link_libraries(transform_points
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(transform_points PRIVATE -g -std=c++11 -Wall -ftree-vectorize ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
  #${OpenMP_CXX_FLAGS}
  )

add_library(generate_point_cloud src/generate_point_cloud.cpp)
target_link_libraries(generate_point_cloud
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(generate_point_cloud PRIVATE -g -std=c++11 -Wall -ftree-vectorize ${SHOW_VECTORIZE_INFO} ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
  ${OpenMP_CXX_FLAGS}
  )



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/egocylindrical.cpp src/cylindricalVisualization.cpp )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   generate_range_image
   transform_points
   generate_point_cloud
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(${PROJECT_NAME}_node PRIVATE -g -std=c++11 -Wall ${OpenMP_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations 
  #${OpenMP_CXX_FLAGS}
  )
  
  
# add_executable(function_tester src/function_tester.cpp)
# target_link_libraries(function_tester
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${OpenMP_CXX_FLAGS}
#    )
# 
#    target_compile_options(function_tester PRIVATE -g -std=c++11 -Wall ${OpenMP_CXX_FLAGS} -ftree-vectorize -fopt-info-vec-all -funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
#   #${OpenMP_CXX_FLAGS}
#   )

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_egocylindrical.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
