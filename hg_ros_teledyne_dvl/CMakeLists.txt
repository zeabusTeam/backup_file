cmake_minimum_required(VERSION 2.8.3)

project(hg_ros_teledyne_dvl)

find_package(catkin REQUIRED COMPONENTS roscpp hg_ros_serial geometry_msgs diagnostic_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp hg_ros_serial geometry_msgs diagnostic_msgs
  DEPENDS 
)

include_directories(        
    ${catkin_INCLUDE_DIRS}    
)

add_executable(dvl_node src/dvl_node.cpp)
target_link_libraries(dvl_node
  ${catkin_LIBRARIES}  
)



