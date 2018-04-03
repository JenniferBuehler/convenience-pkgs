cmake_minimum_required(VERSION 2.8.3)
project(convenience_ros_functions)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  rosbag
  roscpp
  sensor_msgs
  tf
  trajectory_msgs
  eigen_conversions
  convenience_math_functions
  baselib_binding
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
  INCLUDE_DIRS include
  LIBRARIES convenience_ros_functions
  CATKIN_DEPENDS actionlib geometry_msgs rosbag roscpp sensor_msgs
        tf trajectory_msgs eigen_conversions convenience_math_functions baselib_binding
  DEPENDS ${baselib_binding_CATKIN_DEPENDS}
  CFG_EXTRAS export_definitions.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(convenience_ros_functions
  src/ROSFunctions.cpp
  src/RobotInfo.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(convenience_ros_functions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    message(STATUS "Using target_compile_options with cmake version ${CMAKE_VERSION}")
    target_compile_options(convenience_ros_functions PRIVATE ${baselib_binding_DEFINITIONS})
else (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    message(STATUS "Using set_target_properties with cmake version ${CMAKE_VERSION}")
    # not recommended, but works with catkin:
    set_target_properties(convenience_ros_functions PROPERTIES COMPILE_FLAGS ${baselib_binding_DEFINITIONS})
    # This would be better, but when compiling with catkin (and on bloom prerelease), has error
    # "<command-line>:0:1: error: macro names must be identifiers":
    # target_compile_definitions(architecture_log_boost PUBLIC -DBOOST)
    # can look into this at some point, but for now this will work.
endif (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))

## Specify libraries to link a library or executable target against
target_link_libraries(convenience_ros_functions
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS convenience_ros_functions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*"
)
