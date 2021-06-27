cmake_minimum_required(VERSION 3.0.2)
project(joystick_safety)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joystick_safety
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joystick_safety_node src/joystick_safety_node.cpp)


target_link_libraries(joystick_safety_node
   ${catkin_LIBRARIES}
)

