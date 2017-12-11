cmake_minimum_required(VERSION 2.8.3)
project(mission_examples)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	mission_lib_msgs
)

catkin_package(

)

include_directories(
	${catkin_INCLUDE_DIRS}
)

#link_directories(~/catkin_ws/devel/lib)

add_executable(ex_simpleTakeoff src/ex_simpleTakeoff.cpp)
target_link_libraries(ex_simpleTakeoff  ${catkin_LIBRARIES} )
add_dependencies(ex_simpleTakeoff mission_examples_generate_messages_cpp)

add_executable(ex_simpleCheck src/ex_simpleCheck.cpp)
target_link_libraries(ex_simpleCheck  ${catkin_LIBRARIES} )
add_dependencies(ex_simpleCheck mission_examples_generate_messages_cpp)


add_executable(ex_simpleMission src/ex_simpleMission.cpp)
target_link_libraries(ex_simpleMission ${catkin_LIBRARIES} )
add_dependencies(ex_simpleMission mission_examples_generate_messages_cpp)


add_executable(ex_simpleGoto src/ex_simpleGoto.cpp)
target_link_libraries(ex_simpleGoto ${catkin_LIBRARIES} )
add_dependencies(ex_simpleGoto mission_examples_generate_messages_cpp)

add_executable(ex_simpleModeChange src/ex_simpleModeChange.cpp)
target_link_libraries(ex_simpleModeChange ${catkin_LIBRARIES} )
add_dependencies(ex_simpleModeChange mission_examples_generate_messages_cpp)

add_executable(ex_simplePath src/ex_simplePath.cpp)
target_link_libraries(ex_simplePath ${catkin_LIBRARIES} )
add_dependencies(ex_simplePath mission_examples_generate_messages_cpp)

add_executable(ex_simpleGeofence src/ex_simpleGeofence.cpp)
target_link_libraries(ex_simpleGeofence ${catkin_LIBRARIES} )
add_dependencies(ex_simpleGeofence mission_examples_generate_messages_cpp)

add_executable(ex_simpleNoflyZone src/ex_simpleNoflyZone.cpp)
target_link_libraries(ex_simpleNoflyZone ${catkin_LIBRARIES} )
add_dependencies(ex_simpleNoflyZone mission_examples_generate_messages_cpp)


#add_executable(ex_wpGeofence src/ex_wpGeofence.cpp)
#target_link_libraries(ex_wpGeofence mission_lib_common ${catkin_LIBRARIES} )
#add_dependencies(ex_wpGeofence mission_examples_generate_messages_cpp)

#add_executable(ex_wpNoflyZone src/ex_wpNoflyZone.cpp)
#target_link_libraries(ex_wpNoflyZone mission_lib_common ${catkin_LIBRARIES} )
#add_dependencies(ex_wpNoflyZone mission_examples_generate_messages_cpp)







