cmake_minimum_required(VERSION 2.8.3)
project(bzl_modbus)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
  message_generation
)

add_message_files(
	DIRECTORY msg
	FILES Modbus.msg
	ModbusStatus.msg
        ModbusStatusV2.msg
)

#add_service_files(
#        FILES
#        ModbusServiceMsg.srv
#)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS std_msgs roscpp message_runtime
	INCLUDE_DIRS include
)

#include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} lib/modbus/)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include/modbus)

add_library(modbus 
	lib/modbus/modbus.c
	lib/modbus/modbus-data.c
	lib/modbus/modbus-tcp.c
	lib/modbus/modbus-rtu.c
)

#add_executable(modbus_server src/modbus_server.cpp src/ModbusTcpServer.cpp src/ModbusUtils.cpp)
#add_dependencies(modbus_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(modbus_server ${catkin_LIBRARIES} modbus)

add_executable(modbus_server_v2 src/modbus_server_v2.cpp src/ModbusTcpServer.cpp src/ModbusUtils.cpp)
add_dependencies(modbus_server_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(modbus_server_v2 ${catkin_LIBRARIES} modbus)

#add_executable(modbus_client src/modbus_client.cpp src/ModbusUtils.cpp)
#add_dependencies(modbus_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(modbus_client ${catkin_LIBRARIES} modbus)

add_executable(modbus_client_v2 src/modbus_client_v2.cpp)
add_dependencies(modbus_client_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(modbus_client_v2 ${catkin_LIBRARIES} modbus)
