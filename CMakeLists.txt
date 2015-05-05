cmake_minimum_required(VERSION 2.8.3)
project(scalaser)

set(CMAKE_CXX_FLAGS "--std=c++11")

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf
  matlab_cpp_interface
)


generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs  # Or other packages containing msgs
 )

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
target_link_libraries(LaserScanToPointCloud ${catkin_LIBRARIES}  ) #${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}

## add_executable(Hough src/hough.cpp)
##target_link_libraries(Hough ${catkin_LIBRARIES})

add_executable(Angle src/main.cpp
		     src/angle.cpp
		     src/matching.cpp)
target_link_libraries(Angle ${catkin_LIBRARIES})

add_executable(StairModel src/stair_viz.cpp
                          src/stair_model.cpp)
target_link_libraries(StairModel ${catkin_LIBRARIES})

add_executable(ChairModel src/chair_model.cpp)
target_link_libraries(ChairModel ${catkin_LIBRARIES})

add_executable(Laser2Stair src/laser2stair_tf_broadcaster.cpp)
target_link_libraries(Laser2Stair ${catkin_LIBRARIES})


