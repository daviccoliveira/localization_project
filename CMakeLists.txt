cmake_minimum_required(VERSION 3.0.2)
project(localization_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  apriltag_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization_project
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(marker2pointcloud src/marker2pointcloud.cpp)
target_link_libraries(marker2pointcloud ${catkin_LIBRARIES})

add_executable(marker2scan src/marker2scan.cpp)
target_link_libraries(marker2scan ${catkin_LIBRARIES})

install(
  DIRECTORY launch maps models worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)