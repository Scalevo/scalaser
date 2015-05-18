cmake_minimum_required(VERSION 2.8.3)
project(scalaser)

set(CMAKE_CXX_FLAGS "--std=c++11")

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  visualization_msgs
  scalevo_msgs
  tf
  matlab_cpp_interface
  # urdf
)

set(CMAKE_BUILD_TYPE DEBUG)


catkin_package(
  INCLUDE_DIRS include

#  LIBRARIES scalaser
   CATKIN_DEPENDS 
  laser_assembler
  laser_geometry
  roscpp
  
  sensor_msgs
  visualization_msgs
  message_filters
  tf
  matlab_cpp_interface
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS} 
)


add_executable(LaserScanToPointCloud src/LaserScanToPointCloud.cpp)

add_executable(Angle src/main.cpp
		                 src/angle.cpp
		                 src/matching.cpp)

add_executable(StairModel src/stair_viz.cpp
                          src/stair_model.cpp)

add_executable(ChairModel src/chair_model.cpp)

add_executable(Laser2Stair src/laser2stair_tf_broadcaster.cpp)

target_link_libraries(LaserScanToPointCloud ${catkin_LIBRARIES}  )
target_link_libraries(Angle ${catkin_LIBRARIES})
target_link_libraries(Laser2Stair ${catkin_LIBRARIES})
target_link_libraries(ChairModel ${catkin_LIBRARIES})
target_link_libraries(StairModel ${catkin_LIBRARIES})