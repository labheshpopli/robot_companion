cmake_minimum_required(VERSION 2.8.3)
project(raycasting)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
 # sensor_msgs
 # std_msgs
 # image_geometry
 # geometry_msgs
 # message_filters
 # tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES human_motion_capture
   CATKIN_DEPENDS 
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
##add_executable(hand_locator   src/code_cloud_synch.cpp)
##add_executable(image_point_recorder   src/image_point_recorder.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
##add_dependencies(hand_locator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
##add_dependencies(image_point_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(hand_locator
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)

#target_link_libraries(image_point_recorder
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)

