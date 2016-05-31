# This CMake file can be used to find the package. 
# Moreover, it adds variables set to the libraries to
# bind for using the loggers with either implementations of
# c++11 and boost.
# 
# Defines the following cmake variables:
#   logger_binding_LIBRARY_BOOST: the library to bind for the logger
#   logger_binding_LIBRARY_STD: the library to bind for the logger


set(logger_binding_LIBRARY_BOOST logger_binding_log_boost) 
set(logger_binding_LIBRARY_STD logger_binding_log_c++11) 

set(logger_binding_DEFINITIONS baselib_binding_DEFINITIONS) 
set(logger_binding_DEFINITIONS_BOOST baselib_binding_DEFINITIONS_BOOST) 
set(logger_binding_DEFINITIONS_STD baselib_binding_DEFINITIONS_STD) 

get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

## This should be added in case this package provides andy
## targets (libraries or executables).
include(${SELF_DIR}/logger_binding-targets.cmake)

get_filename_component(logger_binding_INCLUDE "${SELF_DIR}/../../include/" ABSOLUTE)
set(logger_binding_INCLUDE_DIRS ${logger_binding_INCLUDE} ${baselib_binding_INCLUDE_DIRS})
