#cmake_minimum_required(VERSION 2.8.3)
#project(m3ros_control)

set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -Wall -DTIMING")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #controller_interface
  controller_manager
  hardware_interface
  controller_interface 
  control_toolbox
  #realtime_tools
)
set(LIBRARIES ${LIBRARIES} ${catkin_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

###############################################################################
# Find RTAI
find_package(RTAI REQUIRED)
set(LIBRARIES ${LIBRARIES} ${RTAI_LIBRARY})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${RTAI_INCLUDE_DIR})

###############################################################################
# Find M3
find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3_INCLUDE_DIR})

###############################################################################
# Find M3RT
find_package(M3rt REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3RT_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR})

###############################################################################
# Find google protobuff
find_package(Protobuf REQUIRED)
set(PROTOBUF_IMPORT_DIRS ${INCLUDE_DIRS})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})

###############################################################################
# Find Eigen
find_package(Eigen3 3.2 REQUIRED)
set(LIBRARIES ${LIBRARIES} ${EIGEN3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Find boost packages
find_package(Boost 1.34 COMPONENTS filesystem system serialization REQUIRED)
set(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${PROJECT_INCLUDE_DIR})
###############################################################################
# Find Yamlcpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
if(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
find_package(YamlCpp  REQUIRED)
set_property(
   DIRECTORY
   PROPERTY COMPILE_DEFINITIONS YAMLCPP_05
   )
endif(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
set(LIBRARIES ${LIBRARIES} ${YAMLCPP_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIR})
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ros_control_rtai
#   CATKIN_DEPENDS roscpp controller_manager hardware_interface controller_interface #control_toolbox
#   #DEPENDS boost ...
#)

include_directories(
  ${INCLUDE_DIRS} 
)

###############################################################################
# Protobuff stuff
file(GLOB ControllerProtoFiles "${PROJECT_PROTO_DIR}/${PROJECT_NAME}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR} ${ControllerProtoFiles})
add_custom_target(${PROJECT_NAME}_cpp_protobuf_output  ALL DEPENDS ${ProtoSources}) # force the generation
PROTOBUF_GENERATE_PYTHON(PySources ${PROJECT_PYTHON_DIR}/${PROJECT_NAME} ${ControllerProtoFiles})
add_custom_target(${PROJECT_NAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}) # To create a directory at build time

## Declare a cpp executable
add_library(${PROJECT_NAME} ros_control_component.cpp factory_proxy.cpp ros_control_component.pb.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})