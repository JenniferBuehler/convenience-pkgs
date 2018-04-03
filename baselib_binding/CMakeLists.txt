# This cmake file provides header files to
# bind a standard interface to either C++11 or boost
# threads and shared pointers.

cmake_minimum_required(VERSION 2.8.11)
project(baselib_binding)


# By default, compiling with catkin is
# enabled. It will only be disabled again
# if CATKIN_DEVEL_PREFIX is not set.
# To enforce compiling *with* catkin
# even if CATKIN_DEVEL_PREFIX is not defined,
# compile with the flag -DENFORCE_CATKIN=true
# This is for example required when compiling on jenkins,
# when it seems that CATKIN_DEVEL_PREFIX is not set.
# So set ENFORCE_CATKIN to true only for the commits
# to jenkins.
set (ENFORCE_CATKIN false)

# Private flag: compiling with catkin is prioritized by default.
# This flag will be set to false if CATKIN_DEVEL_PREFIX
# is empty/not defined, **and** ENFORCE_CATKIN is false.
set (COMPILE_WITH_CATKIN true)

if (ENFORCE_CATKIN)
    message(STATUS "Enforcing compilation with catkin")
endif (ENFORCE_CATKIN)


if (NOT ENFORCE_CATKIN AND NOT CATKIN_DEVEL_PREFIX)
    message(STATUS "Compiling baselib_binding: CATKIN_DEVEL_PREFIX is not defined, so **enforcing compilation with cmake**.")
    set (COMPILE_WITH_CATKIN false)
endif (NOT ENFORCE_CATKIN AND NOT CATKIN_DEVEL_PREFIX)

if (COMPILE_WITH_CATKIN)
    find_package(catkin REQUIRED
        #COMPONENTS roscpp
    )
endif (COMPILE_WITH_CATKIN)

# This CMake file specifies the default options
# to use along with compiler directives required.
# It can be included
# by all other projects which depend on this library.
set (BASELIB_CONFIG_CMAKE baselib_bindingConfigExtras.cmake)

if (COMPILE_WITH_CATKIN)
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
        # LIBRARIES
        # CATKIN_DEPENDS roscpp
        CFG_EXTRAS ${BASELIB_CONFIG_CMAKE}
    )
endif (COMPILE_WITH_CATKIN)

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

install(FILES
    include/${PROJECT_NAME}/Thread.h
    include/${PROJECT_NAME}/SharedPtr.h
    DESTINATION ${INCLUDE_DESTINATION}
)


if (NOT COMPILE_WITH_CATKIN)
    # For cmake-only build: Install the config file.
    # Need to rename the cmake config
    # file as it acts also as a find_package() script,
    # and it needs to go in the lib destination.
    # Cannot keep original name because that conflicts with
    # catkin/jenkins.
    install(FILES
        cmake/${BASELIB_CONFIG_CMAKE}
        DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}/
        RENAME baselib_bindingConfig.cmake
    )
endif (NOT COMPILE_WITH_CATKIN)
