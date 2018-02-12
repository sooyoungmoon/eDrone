cmake_minimum_required(VERSION 2.8.3)
project(eDrone_examples)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	eDrone_msgs
)

catkin_package(
	
	CATKIN_DEPENDS roscpp rospy std_msgs 
	eDrone_msgs 
)

include_directories(
	inc
	${catkin_INCLUDE_DIRS}
)

#link_directories(~/catkin_ws/devel/lib)

add_executable(ex_takeoff src/ex_takeoff.cpp)
add_dependencies (
 ex_takeoff

 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}

)
target_link_libraries(ex_takeoff  ${catkin_LIBRARIES} )




add_executable(ex_check src/ex_check.cpp)
add_dependencies(
ex_check 
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}

)
target_link_libraries(ex_check  ${catkin_LIBRARIES} )

add_executable(ex_mission src/ex_mission.cpp)
add_dependencies(
ex_mission 
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ex_mission ${catkin_LIBRARIES} )


add_executable(ex_goto src/ex_goto.cpp)
add_dependencies(ex_goto
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_goto ${catkin_LIBRARIES} )

add_executable(ex_modeChange src/ex_modeChange.cpp)
add_dependencies(ex_modeChange
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_modeChange ${catkin_LIBRARIES} )

add_executable(ex_geofence src/ex_geofence.cpp)
add_dependencies(ex_geofence
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_geofence ${catkin_LIBRARIES} )

add_executable(ex_noflyZone src/ex_noflyZone.cpp)
add_dependencies(ex_noflyZone
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_noflyZone ${catkin_LIBRARIES} )

add_executable(ex_survey src/ex_survey.cpp)

add_dependencies(ex_survey

 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_survey ${catkin_LIBRARIES} )


add_executable(ex_spraying_mission src/ex_spraying_mission.cpp)

add_dependencies(ex_spraying_mission

 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_spraying_mission ${catkin_LIBRARIES} )

