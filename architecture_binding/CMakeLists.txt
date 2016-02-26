cmake_minimum_required(VERSION 2.8.3)
project(architecture_binding)

# in this CMake file, you should set the switches
# for the choice of architecture (boost or std)
# made for compiling the libraries.
# This CMake file should be included
# by all other projects which depend on this library.
set (ARCH_CONFIG_CMAKE CMakeArchConfig.cmake.in)

include(cmake/CMakeArchConfig.cmake.in)

if (USE_BOOST)
    ## System dependencies are found with CMake's conventions
    find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)
endif (USE_BOOST)

# catkin is optional, and only to be used when added 
# as ROS package and compiling with catkin_make. 
# CATKIN_DEVEL_PREFIX is only defined if building with catkin
# TODO: Would be nicer to use another variable here like
# BUILDING_WITH_CATKIN if something like that exists?
if (CATKIN_DEVEL_PREFIX)
    message("Building architecture_binding with catkin.")
    ## Find catkin macros and libraries
    ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
    ## is used, also find other catkin packages
    find_package(catkin REQUIRED COMPONENTS
      roscpp
    )
else (CATKIN_DEVEL_PREFIX)
    message("Building architecture_binding without catkin.")
endif (CATKIN_DEVEL_PREFIX) 


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if (catkin_FOUND)
    if (USE_BOOST)
        set(CATKIN_DEPEND_LIBS ${Boost_LIBRARIES})
    endif (USE_BOOST)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES architecture_binding_log
        CATKIN_DEPENDS roscpp
        DEPENDS ${CATKIN_DEPEND_LIBS}
        CFG_EXTRAS ${ARCH_CONFIG_CMAKE}
    )
endif (catkin_FOUND)

###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(LOG_LIBRARY_SOURCES
    src/LogBinding.cpp
)

## Declare a C++ library
add_library(architecture_binding_log
    ${LOG_LIBRARY_SOURCES}
)

## Add cmake target dependencies of the library
if (catkin_FOUND)
    add_dependencies(architecture_binding_log ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (catkin_FOUND)

## Declare a C++ executable
# add_executable(architecture_binding_node src/architecture_binding_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(architecture_binding_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
if (catkin_FOUND)
  target_link_libraries(architecture_binding_log
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif (catkin_FOUND)


#############
## Install ##
#############


if (catkin_FOUND) 
    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
    set ( INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set ( LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set ( BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set ( SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else (catkin_FOUND) 
    set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
    set ( LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
    set ( BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
    set ( SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)
endif (catkin_FOUND) 



## Mark executables and/or libraries for installation
install(TARGETS architecture_binding_log 
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${LIB_DESTINATION}
)

install(FILES
    include/${PROJECT_NAME}/LogBinding.h
    include/${PROJECT_NAME}/Thread.h
    include/${PROJECT_NAME}/SharedPtr.h
    DESTINATION ${INCLUDE_DESTINATION}
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_architecture_binding.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
