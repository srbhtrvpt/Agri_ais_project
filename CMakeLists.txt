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
  tf2_ros
  pcl_ros
  pcl_conversions
  cv_bridge
  image_geometry  
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
CATKIN_DEPENDS roscpp tf tf2_ros std_msgs pcl_ros sensor_msgs pcl_conversions image_geometry geometry_msgs nav_msgs visualization_msgs
DEPENDS boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/myproject1.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/myproject1_node.cpp)

add_executable(pcl_integrator_node
  src/pcl_integrator_node.cpp
  src/window.cpp
  )

add_executable(ransac_node src/ransac_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})# ${OpenCV_LIBS})

target_link_libraries(pcl_integrator_node  ${catkin_LIBRARIES})

target_link_libraries(ransac_node  ${catkin_LIBRARIES})# ${PCL_LIBRARIES})

