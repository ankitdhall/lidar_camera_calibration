cmake_minimum_required(VERSION 2.8.3)
project(aruco_mapping)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
             roscpp
             message_generation
             image_transport
             cv_bridge
             tf
             aruco
             visualization_msgs
             camera_calibration_parsers
             lidar_camera_calibration)

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)

include_directories(${PROJECT_SOURCE_DIR}/src/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp
            ${PROJECT_SOURCE_DIR}/src/aruco_mapping.cpp)
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/aruco_mapping.h)

add_message_files(FILES ArucoMarker.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs)
   
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} aruco_mapping_gencpp )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${aruco_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})


 
