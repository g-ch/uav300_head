cmake_minimum_required(VERSION 2.8.3)

project(uav300_head)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
catkin_package()

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  "../../devel/include"
  "include"
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


add_executable(local_msg_show src/local_msg_show.cpp)
target_link_libraries(local_msg_show ${catkin_LIBRARIES} ${PCL_LIBRARIES})




