cmake_minimum_required(VERSION 2.8.3)
project(egocylindrical)

add_compile_options(-Wall -std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  pcl_ros
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
  benchmarking_tools #Probably just temporary to avoid adding any custom dependencies to this package.
)


find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)
find_package(OpenMP)

if(0)
    set(SHOW_VECTORIZE_INFO "-fopt-info-vec-all")
else()
    set(SHOW_VECTORIZE_INFO "")
endif()


if(0) #if on arm...
    set(PLATFORM_FLAGS "PIPS_ON_ARM=arm")
else()
    set(PLATFORM_FLAGS "")
endif()

set(CXX11_FEATURES
    cxx_auto_type
    cxx_constexpr
    cxx_nullptr
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
  #EgoCylinderInfo.msg
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
  cfg/RangeImageGenerator.cfg
  cfg/Filter.cfg
  cfg/RangeImageInflatorGenerator.cfg
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
  LIBRARIES egocylindrical_propagator
  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport roscpp rospy sensor_msgs std_msgs tf tf2 tf2_ros
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

#message(STATUS=WARNING ${CMAKE_CXX_COMPILE_FEATURES})

# This is a separate library partly to reduce compile times, but mostly because it needs to be compiled with certain 'fast-math' optimization options
add_library(range_image_core src/range_image_core.cpp)
add_dependencies(range_image_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(range_image_core PUBLIC ${CXX11_FEATURES})
target_compile_options(range_image_core PRIVATE -ftree-vectorize ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(range_image_core
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

add_library(can_image_core src/can_image_core.cpp)
add_dependencies(can_image_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(can_image_core PUBLIC ${CXX11_FEATURES})
target_compile_options(can_image_core PRIVATE -ftree-vectorize ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(can_image_core
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

##Verified that ${OpenMP_CXX_FLAGS} contains what it should
#message(STATUS=WARNING ${OpenMP_CXX_FLAGS})
  
add_library(point_cloud_core src/point_cloud_core.cpp)
add_dependencies(point_cloud_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(point_cloud_core PUBLIC ${CXX11_FEATURES})
target_compile_options(point_cloud_core PRIVATE -ftree-vectorize ${SHOW_VECTORIZE_INFO} ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(point_cloud_core
   ${catkin_LIBRARIES}
   #${OpenCV_LIBRARIES}
   #${PCL_LIBRARIES}
   #${OpenMP_CXX_FLAGS}    #Not currently needed
   )

add_library(projected_point_cloud_core src/projected_point_cloud_core.cpp)
add_dependencies(projected_point_cloud_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(projected_point_cloud_core PUBLIC ${CXX11_FEATURES})
target_compile_options(projected_point_cloud_core PRIVATE -ftree-vectorize ${SHOW_VECTORIZE_INFO} ${OpenMP_CXX_FLAGS} #-funsafe-math-optimizations -ffinite-math-only -fno-math-errno #-finite-math-only and -fno-trapping-math.
   )
target_link_libraries(projected_point_cloud_core
   ${catkin_LIBRARIES}
   #${OpenCV_LIBRARIES}
   #${PCL_LIBRARIES}
   #${OpenMP_CXX_FLAGS}
   )

add_library(point_transformer src/point_transformer.cpp)
add_dependencies(point_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(point_transformer PUBLIC ${CXX11_FEATURES})
target_compile_options(point_transformer PRIVATE -ftree-vectorize   ${SHOW_VECTORIZE_INFO} -funsafe-math-optimizations -fno-math-errno ${OpenMP_CXX_FLAGS}#-finite-math-only and -fno-trapping-math. -ffinite-math-only 
   )
target_compile_definitions(point_transformer PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(point_transformer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )


  
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

#EGOCYLINDRICAL_PROPAGATOR
add_library(egocylindrical_propagator src/egocylindrical_propagator.cpp)
add_dependencies(egocylindrical_propagator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(egocylindrical_propagator PRIVATE ${SHOW_VECTORIZE_INFO} -ftree-vectorize -funsafe-math-optimizations ${OpenMP_CXX_FLAGS} 
   )
target_compile_definitions(egocylindrical_propagator PRIVATE ${PLATFORM_FLAGS})
target_link_libraries(egocylindrical_propagator
   point_transformer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   #${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

#EGOCYLINDRICAL_PROPAGATOR_NODE
add_executable(egocylindrical_propagator_node src/egocylindrical_propagator_node.cpp)
target_link_libraries(egocylindrical_propagator_node
   egocylindrical_propagator
   ${catkin_LIBRARIES}
   )
   
#EGOCYLINDRICAL_PROPAGATOR_NODELET
add_library(egocylindrical_propagator_nodelet MODULE src/egocylindrical_propagator_nodelet.cpp)
target_link_libraries(egocylindrical_propagator_nodelet
   egocylindrical_propagator
   ${catkin_LIBRARIES}
   )


#POINT_CLOUD_GENERATOR
add_library(point_cloud_generator src/point_cloud_generator.cpp)
target_link_libraries(point_cloud_generator
   point_cloud_core
   ${catkin_LIBRARIES}
   )

#POINT_CLOUD_NODE
add_executable(point_cloud_node src/point_cloud_node.cpp)
target_link_libraries(point_cloud_node
   point_cloud_generator
   ${catkin_LIBRARIES}
   )

#POINT_CLOUD_NODELET
add_library(point_cloud_nodelet MODULE src/point_cloud_nodelet.cpp)
target_link_libraries(point_cloud_nodelet
   point_cloud_generator
   ${catkin_LIBRARIES}
   )


#PROJECTED_POINT_CLOUD_GENERATOR
add_library(projected_point_cloud_generator src/projected_point_cloud_generator.cpp)
target_link_libraries(projected_point_cloud_generator
   projected_point_cloud_core
   ${catkin_LIBRARIES}
   )

#PROJECTED_POINT_CLOUD_NODE
add_executable(projected_point_cloud_node src/projected_point_cloud_node.cpp)
target_link_libraries(projected_point_cloud_node
   projected_point_cloud_generator
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )
  
#PROJECTED_POINT_CLOUD_NODELET
add_library(projected_point_cloud_nodelet MODULE src/projected_point_cloud_nodelet.cpp)
target_link_libraries(projected_point_cloud_nodelet
   projected_point_cloud_generator
   ${catkin_LIBRARIES}
   )


#RANGE_IMAGE_GENERATOR
add_library(range_image_generator src/range_image_generator.cpp)
target_link_libraries(range_image_generator
   range_image_core
   can_image_core
   ${catkin_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

#RANGE_IMAGE_NODE
add_executable(range_image_node src/range_image_node.cpp)
target_link_libraries(range_image_node
   range_image_generator
   ${catkin_LIBRARIES}
   )
  
#RANGE_IMAGE_NODELET
add_library(range_image_nodelet MODULE src/range_image_nodelet.cpp)
target_link_libraries(range_image_nodelet
   range_image_generator
   ${catkin_LIBRARIES}
   )
   
   
# 
# add_library(dedicated_egocylindrical_propagator_nodelet src/dedicated_egocylindrical_propagator.cpp src/dedicated_egocylindrical_propagator_nodelet.cpp) #src/range_image_core.cpp
# 
# target_link_libraries(dedicated_egocylindrical_propagator_nodelet
#    range_image_core
#    #point_transformer
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    #${PCL_LIBRARIES}
#    ${OpenMP_CXX_FLAGS}
#    )
# add_dependencies(dedicated_egocylindrical_propagator_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# target_compile_options(dedicated_egocylindrical_propagator_nodelet PRIVATE -g -std=c++11 -Wall ${SHOW_VECTORIZE_INFO} -fopt-info-vec-optimized-missed -ftree-vectorize -funsafe-math-optimizations ${OpenMP_CXX_FLAGS} -fno-math-errno 
#   )
# target_compile_definitions(dedicated_egocylindrical_propagator_nodelet PRIVATE ${PLATFORM_FLAGS})
# 
#    
#    
# add_executable(dedicated_egocylindrical_propagator_node src/dedicated_egocylindrical_propagator_node.cpp) #src/range_image_core.cpp
# 
# target_link_libraries(dedicated_egocylindrical_propagator_node
#    range_image_core
#    #point_transformer
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    #${PCL_LIBRARIES}
#    ${OpenMP_CXX_FLAGS}
#    )
# add_dependencies(dedicated_egocylindrical_propagator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# target_compile_options(dedicated_egocylindrical_propagator_node PRIVATE -g -std=c++11 -Wall ${SHOW_VECTORIZE_INFO} -fopt-info-vec-optimized-missed -ftree-vectorize -funsafe-math-optimizations ${OpenMP_CXX_FLAGS} -fno-math-errno 
#   )
# target_compile_definitions(dedicated_egocylindrical_propagator_node PRIVATE ${PLATFORM_FLAGS})

   
   
   
#RANGE_IMAGE_CONVERTER
add_library(range_image_converter src/range_image_converter.cpp)
add_dependencies(range_image_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(range_image_converter PUBLIC ${CXX11_FEATURES})
target_link_libraries(range_image_converter
   ${catkin_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )
  
#RANGE_IMAGE_CONVERTER_NODE
add_executable(range_image_converter_node src/range_image_converter_node.cpp)
add_dependencies(range_image_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_image_converter_node
   range_image_converter
   ${catkin_LIBRARIES}
   )


#RANGE_IMAGE_INFLATOR
add_library(range_image_inflator src/range_image_inflator_generator.cpp)
add_dependencies(range_image_inflator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(range_image_inflator PUBLIC ${CXX11_FEATURES})
target_link_libraries(range_image_inflator
   ${catkin_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )
  
##RANGE_IMAGE_INFLATOR_NODE
add_executable(range_image_inflator_node src/range_image_inflator_node.cpp)
add_dependencies(range_image_inflator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_image_inflator_node
   range_image_inflator
   ${catkin_LIBRARIES}
   )

#RANGE_IMAGE_INFLATOR_NODELET
add_library(egocylindrical_range_image_inflator_nodelet MODULE src/range_image_inflator_nodelet.cpp)
add_dependencies(egocylindrical_range_image_inflator_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(egocylindrical_range_image_inflator_nodelet PUBLIC ${CXX11_FEATURES})
target_link_libraries(egocylindrical_range_image_inflator_nodelet
   range_image_inflator
   ${catkin_LIBRARIES}
   )


#CAN_IMAGE_INFLATOR
add_library(can_image_inflator src/can_image_inflator_generator.cpp)
add_dependencies(can_image_inflator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(can_image_inflator PUBLIC ${CXX11_FEATURES})
target_link_libraries(can_image_inflator
   ${catkin_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

##RANGE_IMAGE_INFLATOR_NODE
add_executable(can_image_inflator_node src/can_image_inflator_node.cpp)
add_dependencies(can_image_inflator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(can_image_inflator_node PUBLIC ${CXX11_FEATURES})
target_link_libraries(can_image_inflator_node
   can_image_inflator
   ${catkin_LIBRARIES}
   )

#RANGE_IMAGE_CONVERTER_NODELET
add_library(egocylindrical_range_image_converter_nodelet MODULE src/range_image_converter_nodelet.cpp)
add_dependencies(egocylindrical_range_image_converter_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(egocylindrical_range_image_converter_nodelet
   range_image_converter
   ${catkin_LIBRARIES}
   )

  
#APPROXIMATION_VALIDATION
add_executable(approximation_validation src/approximation_validation.cpp)
add_dependencies(approximation_validation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(approximation_validation PUBLIC ${CXX11_FEATURES})
target_link_libraries(approximation_validation
   ${catkin_LIBRARIES}
   )


add_executable(filter_node src/depth_image_filter.cpp)
add_dependencies(filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_compile_features(filter_node PUBLIC ${CXX11_FEATURES})
target_link_libraries(filter_node
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
