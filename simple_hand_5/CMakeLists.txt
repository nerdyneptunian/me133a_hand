cmake_minimum_required(VERSION 2.8.3)

project(simple_hand_5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  visualization_msgs)

find_package(Eigen3 REQUIRED)

## Uncomment if the package has a setup.py
## catkin_python_setup()

catkin_package()

find_package(roslaunch)

###########
## Build ##
###########

## Specify locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

## Declare the C++ executables
# add_executable(sendjoints  src/sendjoints.cpp)

## Add cmake target dependencies of the executables
# add_dependencies(sendjoints  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(sendjoints  ${catkin_LIBRARIES})


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
