cmake_minimum_required(VERSION 3.0.2)
project(goose_comm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  roslib
  pcl_ros
  pcl_conversions
  goose_comm_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

find_path (PUGIXML_INCLUDE_DIR NAMES pugixml.hpp PATHS ${PUGIXML_HOME}/include)
find_library (PUGIXML_LIBRARY NAMES pugixml PATHS ${PUGIXML_HOME}/lib)
include (FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS (PugiXML DEFAULT_MSG PUGIXML_LIBRARY
PUGIXML_INCLUDE_DIR)

if (PUGIXML_FOUND)
  set (PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
  message (STATUS "PugiXML include = ${PUGIXML_INCLUDE_DIR}")
  message (STATUS "PugiXML library = ${PUGIXML_LIBRARY}")
else (PUGIXML_FOUND)
  message (STATUS "No PugiXML found")
endif(PUGIXML_FOUND)

mark_as_advanced (PUGIXML_LIBRARY PUGIXML_INCLUDE_DIR)


find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES goose_comm_utm_lib
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs roslib
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${GeographicLib_DEFINITIONS})

add_library(goose_comm_utm_lib
  lib/UTM.cpp
)
add_dependencies(goose_comm_utm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(${PROJECT_NAME} src/goose_comm.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  goose_comm_utm_lib
  ${PUGIXML_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/goose_comm_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  goose_comm_utm_lib
  ${PROJECT_NAME}
  ${PUGIXML_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(goose_sensor_data_comm_node src/sensor_data.cpp)
add_dependencies(goose_sensor_data_comm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goose_sensor_data_comm_node
  ${catkin_LIBRARIES}
  goose_comm_utm_lib
  ${PROJECT_NAME}
  ${PUGIXML_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
)
