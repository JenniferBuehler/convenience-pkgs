# This cmake file compiles two versions of a logger
# binding library: one using C++11, the other using boost 
# threads and shared pointers.
#
# The logger binding can be used to switch between 
# different loggers, eg. either using std::cout
# or any other implementation, which also can be
# used either with boost or std c++11.
#
# Compilation of the libraries can be individually
# enabled or disabled by setting the COMPILE_BOOST
# and COMPILE_C++11 flags.
cmake_minimum_required(VERSION 2.8.11)
project(logger_binding)

# By default, compiling with catkin is
# enabled. It will only be disabled again
# if CATKIN_DEVEL_PREFIX is not set.
# To enforce compiling *with* catkin
# even if CATKIN_DEVEL_PREFIX is not defined,
# compile with the flag -DENFORCE_CATKIN=true

# To enforce compiling with catkin in ANY case,
# set ENFORCE_CATKIN to true. This is for example
# required when compiling with jenkins, when
# it seems that CATKIN_DEVEL_PREFIX is not set.
# So set ENFORCE_CATKIN to true only for the commits
# to jenkins.
set (ENFORCE_CATKIN true)

if (ENFORCE_CATKIN)
    message(STATUS "Enforcing compilation with catkin")
endif (ENFORCE_CATKIN)

# to disable compilation of the boost library,
# set this to false
if (NOT DEFINED COMPILE_BOOST)
    set(COMPILE_BOOST true)
endif (NOT DEFINED COMPILE_BOOST)

# to disable compilation of the c++11 library,
# set this to false
if (NOT DEFINED COMPILE_C++11)
    set(COMPILE_C++11 true)
endif (NOT DEFINED COMPILE_C++11)
  
# Private flag: Compiling with catkin is prioritized by default.
# This flag will be set to false if CATKIN_DEVEL_PREFIX
# is empty/not defined, **and** ENFORCE_CATKIN is false.
set (COMPILE_WITH_CATKIN true)

if (NOT ENFORCE_CATKIN AND NOT CATKIN_DEVEL_PREFIX)
    message(STATUS "Compiling logger_binding: CATKIN_DEVEL_PREFIX is not defined, so **enforcing compilation with cmake**.")
    set (COMPILE_WITH_CATKIN false)
endif (NOT ENFORCE_CATKIN AND NOT CATKIN_DEVEL_PREFIX)

# find baselib_binding: if boost is enabled with baselib_binding_USE_BOOST
# (default is true), this will also find Boost and add the cmake variables
# to baselib_binding variables.
if (COMPILE_WITH_CATKIN)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        baselib_binding
    )
else (COMPILE_WITH_CATKIN)
    find_package(baselib_binding REQUIRED)
endif (COMPILE_WITH_CATKIN)

# This CMake file can be used to find this package
set (LOGGER_CONFIG_CMAKE logger_bindingConfigExtras.cmake)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if (COMPILE_WITH_CATKIN)
    if (baselib_binding_USE_BOOST)
        set(CATKIN_GEN_LIBS logger_binding_boost)
    else (baselib_binding_USE_BOOST)
        set(CATKIN_GEN_LIBS logger_binding_c++11)
    endif (baselib_binding_USE_BOOST)
    
    catkin_package(
        INCLUDE_DIRS include ${baselib_binding_INCLUDE_DIRS}
        LIBRARIES ${CATKIN_GEN_LIBS} 
        CATKIN_DEPENDS roscpp  
        DEPENDS ${baselib_binding_CATKIN_DEPENDS}
        CFG_EXTRAS ${LOGGER_CONFIG_CMAKE}
    )
endif (COMPILE_WITH_CATKIN)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${baselib_binding_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(LOG_LIBRARY_SOURCES
    src/LogBinding.cpp
)

if (COMPILE_BOOST)
    add_library(logger_binding_boost SHARED
        ${LOG_LIBRARY_SOURCES}
    )
endif (COMPILE_BOOST)
    
if (COMPILE_C++11)
    add_library(logger_binding_c++11 SHARED
        ${LOG_LIBRARY_SOURCES}
    )
endif (COMPILE_C++11)

message(STATUS "Logger_binding: using compiler definitions: Boost=${baselib_binding_DEFINITIONS_BOOST}, c++11=${baselib_binding_DEFINITIONS_STD}")
if (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    message(STATUS "Using target_compile_options with cmake version ${CMAKE_VERSION}")
    target_compile_options(logger_binding_boost PRIVATE ${baselib_binding_DEFINITIONS_BOOST}) 
    target_compile_options(logger_binding_c++11 PRIVATE ${baselib_binding_DEFINITIONS_STD}) 
else (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    message(STATUS "Using set_target_properties with cmake version ${CMAKE_VERSION}")
    # for set_target_properties, need to build string out of arrays:
    string (REPLACE ";" " " _baselib_binding_DEFINITIONS_BOOST "${baselib_binding_DEFINITIONS_BOOST}")
    string (REPLACE ";" " " _baselib_binding_DEFINITIONS_STD "${baselib_binding_DEFINITIONS_STD}")
    message(STATUS "set_target_properties cmake flags: boost = ${_baselib_binding_DEFINITIONS_BOOST}, c++11 = ${_baselib_binding_DEFINITIONS_STD}")
    # not recommended, but works with catkin:
    set_target_properties(logger_binding_boost PROPERTIES COMPILE_FLAGS ${_baselib_binding_DEFINITIONS_BOOST})
    set_target_properties(logger_binding_c++11 PROPERTIES COMPILE_FLAGS ${_baselib_binding_DEFINITIONS_STD})
    # This would be better, but when compiling with catkin (and on bloom prerelease), has error 
    # "<command-line>:0:1: error: macro names must be identifiers":
    # target_compile_definitions(architecture_log_boost PUBLIC -DBOOST)
    # can look into this at some point, but for now this will work.
endif (NOT (CMAKE_VERSION VERSION_LESS 2.8.12))

## Add cmake target dependencies of the library
if (COMPILE_WITH_CATKIN)
    if (COMPILE_BOOST)
        add_dependencies(logger_binding_boost ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif (COMPILE_BOOST)
    if (COMPILE_C++11)
        add_dependencies(logger_binding_c++11 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif (COMPILE_C++11)
endif (COMPILE_WITH_CATKIN)

## Specify libraries to link a library or executable target against
if (COMPILE_WITH_CATKIN)
    set (LINK_LIBS ${catkin_LIBRARIES})
endif (COMPILE_WITH_CATKIN)
  
if (COMPILE_BOOST)
    target_link_libraries(logger_binding_boost
        ${LINK_LIBS}
        ${baselib_binding_LIBRARIES_BOOST}
    )
endif (COMPILE_BOOST)

if (COMPILE_C++11)
    target_link_libraries(logger_binding_c++11
        ${LINK_LIBS}
        ${baselib_binding_LIBRARIES_STD}
    )
endif (COMPILE_C++11)


#############
## Install ##
#############

if (COMPILE_WITH_CATKIN) 
    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
    set ( INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set ( LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set ( BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set ( SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else (COMPILE_WITH_CATKIN) 
    set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
    set ( LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
    set ( BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
    set ( SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)
endif (COMPILE_WITH_CATKIN) 

if (COMPILE_BOOST)
    set(INSTALL_LIBS logger_binding_boost)
endif (COMPILE_BOOST)
if (COMPILE_C++11)
    set(INSTALL_LIBS ${INSTALL_LIBS} logger_binding_c++11)
endif (COMPILE_C++11)


## Mark executables and/or libraries for installation
install(TARGETS ${INSTALL_LIBS} 
   EXPORT logger_binding-targets 
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${LIB_DESTINATION}
)

install(EXPORT logger_binding-targets
   DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)

install(FILES
    include/${PROJECT_NAME}/LogBinding.h
    include/${PROJECT_NAME}/LogBindingROS.h
    DESTINATION ${INCLUDE_DESTINATION}
)

if (NOT COMPILE_WITH_CATKIN) 
    # For cmake-only build: Install config file.
    # Need to rename the cmake config
    # file as it acts also as a find_package() script,
    # and it needs to go in the lib destination
    # Cannot keep original name because that conflicts with
    # catkin/jenkins
    install(FILES
        cmake/${LOGGER_CONFIG_CMAKE}
        DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}/
        RENAME logger_bindingConfig.cmake
    )
endif (NOT COMPILE_WITH_CATKIN) 

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_logger_binding.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
