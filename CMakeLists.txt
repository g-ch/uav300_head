cmake_minimum_required(VERSION 2.8.3)

project(uav300_head)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
	"../../devel/include"
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(broadcast_and_show src/broadcast_and_show.cpp)
target_link_libraries(broadcast_and_show ${catkin_LIBRARIES} ${OpenCV_LIBS})
