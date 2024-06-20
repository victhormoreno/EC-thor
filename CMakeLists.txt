cmake_minimum_required(VERSION 3.0.2)
project(ecthor)

SET(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

message(STATUS "Welcome to EC-thor! :)")

# OpenMP
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  sensor_msgs
  std_msgs
  tf
  geometry_msgs 
  as_msgs
)

generate_messages(
  DEPENDENCIES
  as_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dbscone
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS OpenMP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
)
  


add_executable(ecthor
  # include/Modules

  src/main.cpp
  
  # modules
  src/modules/Manager.cpp
  src/modules/Processor.cpp
  src/modules/Accumulator.cpp
  src/modules/Detector.cpp
  src/modules/Reconstructor.cpp
  
  # utils
  src/utils/Waggner.cpp
  src/utils/FEC.cpp
  src/utils/ikdTree.cpp
)

set(LINK_LIBRARIES
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${PYTHON_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)

target_link_libraries(ecthor ${LINK_LIBRARIES})

# Mandatory for parallelism (OpenMP)
target_compile_options(ecthor PRIVATE ${OpenMP_FLAGS})
