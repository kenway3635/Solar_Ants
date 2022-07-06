cmake_minimum_required(VERSION 2.8.3)
project(willy_v2)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	message_generation
	roscpp
	rospy
	serial
	std_msgs
	tf
	sensor_msgs
	message_filters
	message_runtime
)

#find_package(Boost REQUIRED COMPONENTS 
#	signals
#)
set(Boost_INCLUDE_DIRS /usr/include/boost)

add_message_files(
    FILES
    Float_Header.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)



catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs serial message_runtime
	INCLUDE_DIRS include
)



 
include_directories(
  	${catkin_INCLUDE_DIRS}
	${serial_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	include
)


add_executable(willy_v2_encoder src/willy_encoder.cpp)
target_link_libraries(willy_v2_encoder ${catkin_LIBRARIES})
add_dependencies(willy_v2_encoder willy_v2_generate_messages_cpp)

add_executable(willy_motor_left src/willy_motor_left.cpp)
target_link_libraries(willy_motor_left ${catkin_LIBRARIES})
add_dependencies(willy_motor_left willy_v2_generate_messages_cpp)

add_executable(willy_motor_right src/willy_motor_right.cpp)
target_link_libraries(willy_motor_right ${catkin_LIBRARIES})
add_dependencies(willy_motor_right willy_v2_generate_messages_cpp)
