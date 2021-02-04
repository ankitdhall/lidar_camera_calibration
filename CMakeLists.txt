cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(lidar_camera_calibration)
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")
# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  cv_bridge
  image_transport
  image_geometry
  velodyne_msgs
  velodyne_pointcloud
  velodyne_pcl
  pcl_ros
)

add_message_files(FILES marker_6dof.msg)
generate_messages(DEPENDENCIES
                  std_msgs
                  )

catkin_package(CATKIN_DEPENDS std_msgs velodyne_pointcloud velodyne_pcl
    DEPENDS eigen opencv)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(find_transform src/find_velodyne_points.cpp src/Corners.cpp) 
target_link_libraries(find_transform ${PCL_LIBRARIES}) 
target_link_libraries(find_transform ${catkin_LIBRARIES})
target_link_libraries(find_transform ${OpenCV_LIBS})
