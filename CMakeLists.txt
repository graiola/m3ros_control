###############################################################################
# States that CMake required version must be greater than 2.8
cmake_minimum_required(VERSION 2.8)

###############################################################################
# Project name is not mandatory, but you should use it
project(m3ros_control)

###############################################################################
# Set the PROJECT_ROOT variable
set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

###############################################################################
# Set where to find the FindXXX.cmake
#set(CMAKE_MODULE_PATH "${PROJECT_ROOT}/../meka-cmake-modules/Modules/" ${CMAKE_MODULE_PATH})

###############################################################################
# Set all bunch of usefull variables
set(PROJECT_INCLUDE_DIR ${PROJECT_ROOT}/include)
set(PROJECT_PROTO_DIR ${PROJECT_ROOT}/proto)
set(PROJECT_SRC_DIR ${PROJECT_ROOT}/src)
set(PROJECT_INSTALL_BIN ${PROJECT_ROOT}/bin)
set(PROJECT_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT}/bin)
set(PROJECT_INSTALL_LIB ${PROJECT_ROOT}/lib)
set(PROJECT_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_ROOT}/lib)
set(PROJECT_PYTHON_DIR ${PROJECT_ROOT}/python)

###############################################################################
# Forcing the generation dirs to be local
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})

###############################################################################
# Include directories
include_directories(
    ${PROJECT_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}/src # Include this directory for the generated protobufs headers
)

###############################################################################
# Add the main project src dir that contains all the others
add_subdirectory(src/m3ros_control)
