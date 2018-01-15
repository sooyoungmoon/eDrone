cmake_minimum_required(VERSION 2.8.3)
project(eDrone_examples)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	eDrone_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs eDrone_msgs 
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

#link_directories(~/catkin_ws/devel/lib)

add_executable(ex_simpleTakeoff src/ex_simpleTakeoff.cpp)
add_dependencies (
 ex_simpleTakeoff

 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}

)
target_link_libraries(ex_simpleTakeoff  ${catkin_LIBRARIES} )




add_executable(ex_simpleCheck src/ex_simpleCheck.cpp)
add_dependencies(
ex_simpleCheck 
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}

)
target_link_libraries(ex_simpleCheck  ${catkin_LIBRARIES} )

add_executable(ex_simpleMission src/ex_simpleMission.cpp)
add_dependencies(
ex_simpleMission 
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ex_simpleMission ${catkin_LIBRARIES} )


add_executable(ex_simpleGoto src/ex_simpleGoto.cpp)
add_dependencies(ex_simpleGoto
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_simpleGoto ${catkin_LIBRARIES} )

add_executable(ex_simpleModeChange src/ex_simpleModeChange.cpp)
add_dependencies(ex_simpleModeChange
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_simpleModeChange ${catkin_LIBRARIES} )

add_executable(ex_simplePath src/ex_simplePath.cpp)
add_dependencies(ex_simplePath
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )

target_link_libraries(ex_simplePath ${catkin_LIBRARIES} )

add_executable(ex_simpleGeofence src/ex_simpleGeofence.cpp)
add_dependencies(ex_simpleGeofence
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_simpleGeofence ${catkin_LIBRARIES} )

add_executable(ex_simpleNoflyZone src/ex_simpleNoflyZone.cpp)
add_dependencies(ex_simpleNoflyZone
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_simpleNoflyZone ${catkin_LIBRARIES} )

add_executable(ex_simpleSurvey src/ex_simpleSurvey.cpp)

add_dependencies(ex_simpleSurvey

 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_simpleSurvey ${catkin_LIBRARIES} )


