cmake_minimum_required(VERSION 2.8.3)
project(ball_orbit_estimator)

find_package(catkin REQUIRED COMPONENTS
  image_transport nodelet roscpp sensor_msgs
  opencv_apps cv_bridge
  )

catkin_package(CATKIN_DEPENDS
  nodelet roscpp sensor_msgs cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_library(SyncStereoImagesNodelet src/message_filter_stereo.cpp)
target_link_libraries(SyncStereoImagesNodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_library(OrbitEstimationNodelet src/ekf_orbit_estimation.cpp)
target_link_libraries(OrbitEstimationNodelet ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(TARGETS
  SyncStereoImagesNodelet
  OrbitEstimationNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${POINTGREY_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
