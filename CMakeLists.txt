cmake_minimum_required(VERSION 2.8.3)
project(myproject1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  cv_bridge
  image_geometry  
  tf2_msgs
  tf2_sensor_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

#find_package(OpenCV REQUIRED)
#find_package(PCL 1.3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


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
# INCLUDE_DIRS include
# LIBRARIES myproject1
CATKIN_DEPENDS roscpp tf tf2 tf2_ros std_msgs pcl_ros sensor_msgs pcl_conversions image_geometry geometry_msgs nav_msgs visualization_msgs tf2_msgs tf2_sensor_msgs
DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 EXTERNAL
)

## Declare a C++ library
add_library(pcl_integrator_lib
   src/window.cpp
   src/pcl_integrator.cpp
)

target_link_libraries(pcl_integrator_lib
  ${catkin_LIBRARIES}
  )

add_library(pcl_segmentor_lib
   src/pcl_segmentor.cpp
)
target_link_libraries(pcl_segmentor_lib ${catkin_LIBRARIES})# ${PCL_LIBRARIES})

add_library(pcl_coloriser_lib
   src/pcl_coloriser.cpp
)
target_link_libraries(pcl_coloriser_lib ${catkin_LIBRARIES})# ${OpenCV_LIBS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(pcl_segmentor_node src/pcl_segmentor_node.cpp)
target_link_libraries(pcl_segmentor_node ${catkin_LIBRARIES}
  pcl_segmentor_lib)


add_executable(pcl_coloriser_node src/pcl_coloriser_node.cpp)
target_link_libraries(pcl_coloriser_node ${catkin_LIBRARIES}
  pcl_coloriser_lib)


add_executable(pcl_integrator_node
  src/pcl_integrator_node.cpp
  )
target_link_libraries(pcl_integrator_node
  ${catkin_LIBRARIES}
  pcl_integrator_lib
  )

add_executable(pcl_integrator_node_offline
  src/pcl_integrator_node_offline.cpp
  )
target_link_libraries(pcl_integrator_node_offline
  ${catkin_LIBRARIES}
  pcl_integrator_lib
  )
