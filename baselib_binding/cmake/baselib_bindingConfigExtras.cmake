# This CMake file specifies compiler definitions required
# and the libraries to bind for either implementations of
# c++11 and boost.
#
# Defines the following cmake variables:
#   baselib_binding_DEFINITIONS_BOOST: the compile definitions you need to use for your targets
#   baselib_binding_DEFINITIONS_STD: the compile definitions you need to use for your targets
#   baselib_binding_DEFINITIONS: based on the variable baselib_binding_USE_BOOST on the cache, contains
#                                the definitions for *either* boost *or* std. If boost was not found,
#                                baselib_binding_USE_BOOST is set to false in any case.
#   baselib_binding_LIBRARIES_BOOST: The boost libraries required for the baselib
#   baselib_binding_LIBRARIES_STD: The stdc++ libraries, if any, required for the baselib
#   baselib_binding_LIBRARIES: based on the variable baselib_binding_USE_BOOST on the cache, contains
#                              the libraries for *either* boost *or* std (could be empty)
#   baselib_binding_CATKIN_DEPENDS: libraries to put as DEPENDS into catkin_package(), if catkin is used.

message(STATUS "Finding baselib_binding with baselib_bindingConfig.cmake (extras)")

set(baselib_binding_DEFINITIONS_BOOST -DUSE_BOOST)
set(baselib_binding_DEFINITIONS_STD -DUSE_C11 -std=c++11)

# When using catkin, the include dirs are determined by catkin itself
if (NOT CATKIN_DEVEL_PREFIX)
    get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
    # message("$$$$$$$ ${CMAKE_CURRENT_LIST_FILE} SELF_DIR = ${SELF_DIR}")
    get_filename_component(baselib_binding_INCLUDE_DIRS "${SELF_DIR}/../../include/" ABSOLUTE)
    ## This should be added in case this package provides any
    ## targets (libraries or executables) at some point.
    # include(${SELF_DIR}/baselib_binding-targets.cmake)
endif (NOT CATKIN_DEVEL_PREFIX)

## Decides whether the baselib_binding implementation
## for Boost OR for std c++11 is to be used.
set(baselib_binding_USE_BOOST false CACHE BOOL
    "Use Boost threads/shared pointers. If false, std c++11 instead")

## Some packages may need to know the variable for BOTH
## Boost and c++11, eg. to generating separate libraries
## using different implementations. So the variables should still
## be provided for both baselib choices, which involves
## a find_package(Boost) without a REQUIRED flag.
find_package(Boost COMPONENTS system thread)
# message(STATUS "Found boost libraries: ${Boost_FOUND}")
if (NOT Boost_FOUND)
    message(STATUS "Boost not found, enforcing baselib_binding_USE_BOOST=false")
    set (baselib_binding_USE_BOOST false)
else (NOT Boost_FOUND)
    set(baselib_binding_LIBRARIES_BOOST ${Boost_LIBRARIES})
endif (NOT Boost_FOUND)

# Set the convenience variables based on the choice of boost/std
if (baselib_binding_USE_BOOST)
    message(STATUS "baselib_binding: using Boost threads/shared pointers")
    set(baselib_binding_CATKIN_DEPENDS boost)
    set(baselib_binding_DEFINITIONS ${baselib_binding_DEFINITIONS} ${baselib_binding_DEFINITIONS_BOOST})
    set(baselib_binding_LIBRARIES ${baselib_binding_LIBRARIES} "${baselib_binding_LIBRARIES_BOOST}")
else (baselib_binding_USE_BOOST)
    message(STATUS "baselib_binding: using c++11 threads/shared pointers")
    set(baselib_binding_DEFINITIONS ${baselib_binding_DEFINITIONS} ${baselib_binding_DEFINITIONS_STD})
endif (baselib_binding_USE_BOOST)
