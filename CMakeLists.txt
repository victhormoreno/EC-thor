cmake_minimum_required(VERSION 3.0.2)
project(ecthor)

add_compile_options(-std=c++17)

message(STATUS "Welcome to EC-thor! :)")

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
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
else()
  message(WARNING "OPENMP NOT FOUND, multithreaded disabled")
endif()


add_executable(ecthor
  include/Modules

  # Pipelines
  src/main.cpp

  src/Modules/Manager.cpp
  src/Modules/GroundSegmenter.cpp
  src/Modules/Accumulator.cpp
  src/Modules/ObjectDetector.cpp
  src/Modules/Reconstructor.cpp

  src/Objects/Params.cpp
  src/Objects/Cluster.cpp
  
  src/Utils/Preproc.cpp
  src/Utils/Ransac.cpp
  src/Utils/FEC.cpp
  src/Utils/ikdTree.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)

target_link_libraries(ecthor ${catkin_LIBRARIES})

# Mandatory for parallelism (OpenMP)
target_compile_options(ecthor PRIVATE ${OpenMP_FLAGS})
