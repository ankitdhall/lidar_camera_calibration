cmake_minimum_required(VERSION 2.8.3)
project(aruco)

find_package(catkin)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(aruco
  src/aruco/arucofidmarkers.cpp
  src/aruco/cvdrawingutils.cpp
  src/aruco/cameraparameters.cpp
  src/aruco/board.cpp
  src/aruco/marker.cpp
  src/aruco/boarddetector.cpp
  src/aruco/markerdetector.cpp
)
target_link_libraries(aruco
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(optimalmarkers
  src/aruco/aruco_selectoptimalmarkers.cpp)
target_link_libraries(optimalmarkers aruco ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS aruco optimalmarkers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

