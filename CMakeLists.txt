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
  nodelet
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
  dynamic_reconfigure
)


find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)
find_package(OpenMP)

if(1)
    set(SHOW_VECTORIZE_INFO "-fopt-info-vec-optimized")
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

generate_dynamic_reconfigure_options(
  cfg/Propagator.cfg
  )

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
  INCLUDE_DIRS include
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



# This is a separate library partly to reduce compile times, but mostly because it needs to be compiled with certain 'fast-math' optimization options
add_library(range_image_core src/range_image_core.cpp)
target_link_libraries(range_image_core
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(range_image_core PRIVATE -g -std=c++11 -Wall -ftree-vectorize ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
  #${OpenMP_CXX_FLAGS}
  )
  
add_dependencies(range_image_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


  
##Verified that ${OpenMP_CXX_FLAGS} contains what it should
#message(STATUS=WARNING ${OpenMP_CXX_FLAGS})
  
add_library(point_cloud_core src/point_cloud_core.cpp)
target_link_libraries(point_cloud_core
   ${catkin_LIBRARIES}
   #${OpenCV_LIBRARIES}
   #${PCL_LIBRARIES}
   #${OpenMP_CXX_FLAGS}
   )

target_compile_options(point_cloud_core PRIVATE -g -std=c++11 -Wall -ftree-vectorize ${SHOW_VECTORIZE_INFO} ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
  ${OpenMP_CXX_FLAGS}
  )
  
add_dependencies(point_cloud_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(point_transformer src/point_transformer.cpp)
target_link_libraries(point_transformer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

   #-fopt-info-vec-all
target_compile_options(point_transformer PRIVATE -g -std=c++11 -Wall -ftree-vectorize   ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -fno-math-errno   ${OpenMP_CXX_FLAGS}#-finite-math-only and -fno-trapping-math. -ffinite-math-only 

  )
  
add_dependencies(point_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#${PROJECT_NAME}_



## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(egocylindrical_propagator src/egocylindrical_propagator.cpp)

target_link_libraries(egocylindrical_propagator
   point_transformer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   #${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )
add_dependencies(egocylindrical_propagator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_options(egocylindrical_propagator PRIVATE -g -std=c++11 -Wall ${SHOW_VECTORIZE_INFO} -ftree-vectorize -funsafe-math-optimizations ${OpenMP_CXX_FLAGS} 
  )

  
  
  
  
  
#EGOCYLINDRICAL_PROPAGATOR_NODE
add_executable(egocylindrical_propagator_node src/egocylindrical_propagator_node.cpp)

target_link_libraries(egocylindrical_propagator_node
   egocylindrical_propagator
   ${OpenMP_CXX_FLAGS}
   ${catkin_LIBRARIES}
   )
   
target_compile_options(egocylindrical_propagator_node PRIVATE -g -std=c++11 -Wall)


add_library(egocylindrical_propagator_nodelet MODULE src/egocylindrical_propagator_nodelet.cpp)

target_link_libraries(egocylindrical_propagator_nodelet
   egocylindrical_propagator
   ${OpenMP_CXX_FLAGS}
   ${catkin_LIBRARIES}
   )

target_compile_options(egocylindrical_propagator_nodelet PRIVATE -g -std=c++11 -Wall)




#POINT_CLOUD_GENERATOR
add_library(point_cloud_generator src/point_cloud_generator.cpp)

#TODO: Add dependencies as needed
add_dependencies(point_cloud_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_cloud_generator
   point_cloud_core
   ${catkin_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(point_cloud_generator PRIVATE -std=c++11 -Wall #${OpenMP_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations 
  #${OpenMP_CXX_FLAGS}
  )


#POINT_CLOUD_NODE
add_executable(point_cloud_node src/point_cloud_node.cpp)

#TODO: Add only necessary dependencies
add_dependencies(point_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_cloud_node
   point_cloud_generator
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(point_cloud_node PRIVATE -std=c++11 -Wall #${OpenMP_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations 
  #${OpenMP_CXX_FLAGS}
  )
  
  
#POINT_CLOUD_NODELET
add_library(point_cloud_nodelet MODULE src/point_cloud_nodelet.cpp)

target_compile_options(point_cloud_nodelet PRIVATE -std=c++11 -Wall #${OpenMP_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations 
  #${OpenMP_CXX_FLAGS}
  )

target_link_libraries(point_cloud_nodelet
   point_cloud_generator
   ${catkin_LIBRARIES}
   )


#RANGE_IMAGE_GENERATOR
add_library(range_image_generator src/range_image_generator.cpp)

#TODO: Add dependencies as needed
add_dependencies(range_image_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(range_image_generator
   range_image_core
   ${catkin_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

target_compile_options(range_image_generator PRIVATE -std=c++11 -Wall #${OpenMP_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations 
  #${OpenMP_CXX_FLAGS}
  )

#RANGE_IMAGE_NODE
add_executable(range_image_node src/range_image_node.cpp)

#TODO: Add dependencies as needed
add_dependencies(range_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(range_image_node
   range_image_generator
   ${catkin_LIBRARIES}
   )

target_compile_options(range_image_node PRIVATE -std=c++11 -Wall #${OpenMP_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations 
  #${OpenMP_CXX_FLAGS}
  )
  
  
#RANGE_IMAGE_NODELET
add_library(range_image_nodelet MODULE src/range_image_nodelet.cpp)

target_link_libraries(range_image_nodelet
   range_image_generator
   ${catkin_LIBRARIES}
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
