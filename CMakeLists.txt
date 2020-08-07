cmake_minimum_required(VERSION 3.10.2)
project(ugl_ros)

find_package(catkin REQUIRED 
  COMPONENTS
    geometry_msgs
    tf2_geometry_msgs
)

find_package(ugl REQUIRED)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor 
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wnull-dereference
    -Wmisleading-indentation
    -Wno-deprecated-copy
)

include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)

enable_sanitizers(project_options)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  # LIBRARIES 
  # CATKIN_DEPENDS
  # DEPENDS
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_convert SHARED
  src/convert_tf2.cpp
)

target_include_directories(${PROJECT_NAME}_convert
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_convert
  PUBLIC
    ugl::math
    ${catkin_LIBRARIES}
  PRIVATE
    project_options
    project_warnings
)
