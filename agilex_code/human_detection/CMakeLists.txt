cmake_minimum_required(VERSION 3.0.2)
project(human_detection)

add_compile_options(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenCV REQUIRED)

find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    sensor_msgs
    message_filters  
    nodelet
    pcl_ros
    roscpp
    sensor_msgs
    message_generation
)

########################
## message generation ##
########################
add_message_files(FILES
  Cluster.msg
  ClusterArray.msg
  Track.msg
  TrackArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
  

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(hdl_people_detection_nodelet
  apps/hdl_people_detection_nodelet.cpp
  src/kidono_human_classifier.cpp
  src/people_detector.cpp
)
target_link_libraries(hdl_people_detection_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_people_detection_nodelet hdl_people_tracking_gencpp)

add_library(hdl_people_tracking_nodelet apps/hdl_people_tracking_nodelet.cpp)
target_link_libraries(hdl_people_tracking_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_people_tracking_nodelet hdl_people_tracking_gencpp)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(sync_msgs ${SRCS})
add_dependencies(sync_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sync_msgs ${catkin_LIBRARIES})