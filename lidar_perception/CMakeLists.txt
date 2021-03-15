cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(lidar_perception)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    pcl_ros
    tf
)

catkin_package(

)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    src
)

add_executable(lidar_perception_node 
    src/lidar_perception_node.cpp
    src/lidar_perception.cpp
)
add_dependencies(lidar_perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(lidar_perception_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})
