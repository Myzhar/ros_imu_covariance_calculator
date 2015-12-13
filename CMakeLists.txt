cmake_minimum_required(VERSION 2.8.7)

project(ros_imu_covariance_calculator)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

#########################################################
# INCLUDES
set(INC_PATH
    ${ros_imu_covariance_calculator_SOURCE_DIR}/src)

include_directories(${INC_PATH})
#########################################################

#########################################################
# SOURCES
file(GLOB_RECURSE SRC *.cpp)
# HEADERS
file(GLOB_RECURSE HDR *.h)
#########################################################

#########################################################
#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
#########################################################


#########################################################
# EXECUTABLE
add_executable(
    imu_cov_calc_node
    ${HDR}
    ${SRC}
)

target_link_libraries(
    imu_cov_calc_node
    ${catkin_LIBRARIES} 
)
#########################################################
