# This cmake file compiles two versions of a library to 
# bind a standard interface to either C++11 or boost 
# threads and shared pointers. It also includes a 
# library to use a Logger, either using std::cout
# or any other implementation, which also can be
# used either with boost or std c++11.
#
# When using the headers, to use Boost define
# -DUSE_BOOST as compile flag and if you want
# to use the logger, additionally link the library
# architecture_binding_log_boost.
# To use c++11 instead, use compile flag -std=c++11
# and link the library architecture_binding_log_c++11
# for the logger. Default options can be enabled
# by including CMakeArchConfig.cmake.in, which is
# installed in the share/ destination.
#
# The libraries can be compiled with or without
# C++/Boost support by setting the COMPILE_BOOST
# and COMPILE_C++11 flags.

cmake_minimum_required(VERSION 2.8.11)
project(architecture_binding)

# to disable compilation of the boost library,
# set this to false
if (NOT COMPILE_BOOST)
    set(COMPILE_BOOST true)
endif (NOT COMPILE_BOOST)

# to disable compilation of the c++11 library,
# set this to false
if (NOT COMPILE_C++11)
    set(COMPILE_C++11 true)
endif (NOT COMPILE_C++11)
   
if (COMPILE_BOOST)
    ## System dependencies are found with CMake's conventions
    find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)
endif (COMPILE_BOOST)

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

# This CMake file specifies the default options
# to use along with compiler directives required.
# It can be included
# by all other projects which depend on this library.
set (ARCH_CONFIG_CMAKE CMakeArchConfig.cmake.in)
include(cmake/CMakeArchConfig.cmake.in)

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
    if (COMPILE_BOOST)
        set(CATKIN_DEPEND_LIBS ${Boost_LIBRARIES})
        set(CATKIN_GEN_LIBS architecture_binding_log_boost)
    endif (COMPILE_BOOST)
    if (COMPILE_C++11)
        set(CATKIN_GEN_LIBS ${CATKIN_GEN_LIBS} architecture_binding_log_c++11)
    endif (COMPILE_C++11)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${CATKIN_GEN_LIBS} 
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
  ${Boost_INCLUDE_DIRS} # will be empty anyway if not using boost
  ${catkin_INCLUDE_DIRS} # will be empty anyway if not compiling with catkin
)

set(LOG_LIBRARY_SOURCES
    src/LogBinding.cpp
)

if (COMPILE_BOOST)
    add_library(architecture_binding_log_boost
        ${LOG_LIBRARY_SOURCES}
    )
endif (COMPILE_BOOST)
    
if (COMPILE_C++11)
    add_library(architecture_binding_log_c++11
        ${LOG_LIBRARY_SOURCES}
    )
endif (COMPILE_C++11)

if ((CMAKE_VERSION VERSION_GREATER 2.8.12) OR (CMAKE_VERSION VERSION_EQUAL 2.8.12))
    message("Using target_compile_options with cmake version ${CMAKE_VERSION}")
    target_compile_options(architecture_binding_log_boost PRIVATE -DUSE_BOOST) 
    target_compile_options(architecture_binding_log_c++11 PRIVATE -std=c++11) 
else ((CMAKE_VERSION VERSION_GREATER 2.8.12) OR (CMAKE_VERSION VERSION_EQUAL 2.8.12))
    message("Using set_target_properties with cmake version ${CMAKE_VERSION}")
    # not recommended, but works with catkin:
    set_target_properties(architecture_binding_log_boost PROPERTIES COMPILE_FLAGS -DUSE_BOOST)
    set_target_properties(architecture_binding_log_c++11 PROPERTIES COMPILE_FLAGS -std=c++11)
    # This would be better, but when compiling with catkin (and on bloom prerelease), has error 
    # "<command-line>:0:1: error: macro names must be identifiers":
    # target_compile_definitions(architecture_log_boost PUBLIC -DBOOST)
    # can look into this at some point, but for now this will work.
endif ((CMAKE_VERSION VERSION_GREATER 2.8.12) OR (CMAKE_VERSION VERSION_EQUAL 2.8.12))

## Add cmake target dependencies of the library
if (catkin_FOUND)
    if (COMPILE_BOOST)
        add_dependencies(architecture_binding_log_boost ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif (COMPILE_BOOST)
    if (COMPILE_C++11)
        add_dependencies(architecture_binding_log_c++11 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif (COMPILE_C++11)
endif (catkin_FOUND)

## Declare a C++ executable
# add_executable(architecture_binding_node src/architecture_binding_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(architecture_binding_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
if (catkin_FOUND)
    set (LINK_LIBS ${catkin_LIBRARIES})
endif (catkin_FOUND)
  
if (COMPILE_BOOST)
    target_link_libraries(architecture_binding_log_boost
        ${LINK_LIBS}
        ${Boost_LIBRARIES}
    )
endif (COMPILE_BOOST)
if (COMPILE_C++11)
    target_link_libraries(architecture_binding_log_c++11
        ${LINK_LIBS}
    )
endif (COMPILE_C++11)


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

if (COMPILE_BOOST)
    set(INSTALL_LIBS architecture_binding_log_boost)
endif (COMPILE_BOOST)
if (COMPILE_C++11)
    set(INSTALL_LIBS ${INSTALL_LIBS} architecture_binding_log_c++11)
endif (COMPILE_C++11)


## Mark executables and/or libraries for installation
install(TARGETS ${INSTALL_LIBS} 
   EXPORT ${PROJECT_NAME}Config 
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${LIB_DESTINATION}
)

install(EXPORT ${PROJECT_NAME}Config
   DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)

install(FILES
    include/${PROJECT_NAME}/LogBinding.h
    include/${PROJECT_NAME}/Thread.h
    include/${PROJECT_NAME}/SharedPtr.h
    DESTINATION ${INCLUDE_DESTINATION}
)

install(FILES
    cmake/CMakeArchConfig.cmake.in
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
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
