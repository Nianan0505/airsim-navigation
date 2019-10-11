cmake_minimum_required(VERSION 2.8.3)
project(location)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Add support for C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall ${CMAKE_CXX_FLAGS}")
#find_package(OpenCV 3 REQUIRED )

#find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  velodyne_msgs
  tf
  lidar_correction
  receive_xsens
  ndt_omp
)

include_directories(include ${catkin_INCLUDE_DIRS})


include_directories(${catkin_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR}/include)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES location
  CATKIN_DEPENDS  geometry_msgs  nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf velodyne_msgs
  ndt_omp
)
###########
## Build ##
###########



add_executable(locationV2 ${PROJECT_SOURCE_DIR}/src/locationV2.cpp )
target_link_libraries(locationV2 ${catkin_LIBRARIES})
include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(location_imuV2 ${PROJECT_SOURCE_DIR}/src/location_imuV2.cpp )
target_link_libraries(location_imuV2 ${catkin_LIBRARIES})
include_directories(include ${catkin_INCLUDE_DIRS} )





