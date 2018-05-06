cmake_minimum_required(VERSION 2.8.3)

project(modbus_ascii_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation sensor_msgs)
#find_package(Boost REQUIRED COMPONENTS system chrono)

add_service_files(
  FILES
  BatteryInfo.srv
  SendCommand.srv
  IOCommand.srv
  ReceiveStatus.srv
  GPIOStatus.srv
  BarometerMode.srv
  DeviceDiscovered.srv
  MBBitRegisterStatus.srv
  MBWordRegisterStatus.srv
  MBBitRegisterCommand.srv
  MBWordRegisterCommand.srv
)

add_message_files(
  FILES
  Switch.msg
  MBGetRegisterName.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(     
	INCLUDE_DIRS ../modbus-ascii/master-device
	CATKIN_DEPENDS roscpp   message_runtime
	DEPENDS Boost 
	LIBRARIES mblib
)

include_directories(
	../modbus-ascii/master-device/
	${Boost_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS}
)

add_library(mblib 
	../modbus-ascii/master-device/mblib/MBDatalink.cpp
	../modbus-ascii/master-device/mblib/MBDatatypes.cpp
	../modbus-ascii/master-device/mblib/MBDevice.cpp	
)

# add_executable(mbserver
# 	../modbus-ascii/master-device/master/main.cpp
# )
# target_link_libraries(mbserver mblib)

add_executable(zeabus_modbus_master
	src/zeabus_modbus_master.cpp
)
add_executable(io_and_pressure
	src/io_and_pressure.cpp
)
target_link_libraries(zeabus_modbus_master mblib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(io_and_pressure mblib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(zeabus_modbus_master modbus_ascii_ros_gencpp)
add_dependencies(io_and_pressure modbus_ascii_ros_gencpp)  
