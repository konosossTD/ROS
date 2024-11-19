cmake_minimum_required(VERSION 3.0.2)
project(orbslam)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES orbslam
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)
add_executable(usb_cam_publisher src/usb_cam_publisher.cpp)
add_executable(cali src/cali.cpp)
add_executable(verify src/verify.cpp)
target_link_libraries( usb_cam_publisher ${catkin_LIBRARIES})
target_link_libraries(cali  ${catkin_LIBRARIES})
target_link_libraries(verify  ${catkin_LIBRARIES})

# add_executable(usb_cam_publisher src/usb_cam_publisher.cpp)

# target_link_libraries(usb_cam_publisher
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}

# )
