cmake_minimum_required(VERSION 3.0.2)
project(rm_ctrl)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        quadrotor_msgs
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES map_simulator
        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        fmt::fmt
        )

add_executable(rm_ctrl_node
        src/rm_ctrl_node.cpp
        )
target_link_libraries(rm_ctrl_node
        ${THIRD_PARTY}
        )

