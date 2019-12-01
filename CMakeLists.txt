cmake_minimum_required(VERSION 2.8.3)

project(uav300_head)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}  
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(tf_broadcaster src/tf_broadcast.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
