cmake_minimum_required(VERSION 2.8.3)
project(arm_components_name_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  convenience_math_functions
  baselib_binding 
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES arm_components_name_manager
    CATKIN_DEPENDS roscpp sensor_msgs convenience_math_functions baselib_binding 
#   DEPENDS system_lib
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
add_library(arm_components_name_manager
  src/ArmComponentsNameManager.cpp
  src/ArmJointStateSubscriber.cpp
)

if (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    message(STATUS "Using target_compile_options with cmake version ${CMAKE_VERSION}")
    target_compile_options(arm_components_name_manager PRIVATE ${baselib_binding_DEFINITIONS}) 
else (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    message(STATUS "Using set_target_properties with cmake version ${CMAKE_VERSION}")
    # not recommended, but works with catkin:
    set_target_properties(arm_components_name_manager PROPERTIES COMPILE_FLAGS ${baselib_binding_DEFINITIONS})
    # This would be better, but when compiling with catkin (and on bloom prerelease), has error 
    # "<command-line>:0:1: error: macro names must be identifiers":
    # target_compile_definitions(architecture_log_boost PUBLIC -DBOOST)
    # can look into this at some point, but for now this will work.
endif (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(arm_components_name_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arm_components_name_manager
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS arm_components_name_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arm_components_name_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
