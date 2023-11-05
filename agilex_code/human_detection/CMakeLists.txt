cmake_minimum_required(VERSION 3.0.2)
project(human_detection)

    add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    sensor_msgs
    message_filters
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    message_filters
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(sync_msgs ${SRCS})
add_dependencies(sync_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sync_msgs ${catkin_LIBRARIES})