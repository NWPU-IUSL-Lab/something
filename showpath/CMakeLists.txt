cmake_minimum_required(VERSION 3.0.2)
project(showpath)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)



catkin_package(

)



include_directories(

  ${catkin_INCLUDE_DIRS}
)



add_executable(showpath src/showpath.cpp)
target_link_libraries(showpath ${catkin_LIBRARIES})

