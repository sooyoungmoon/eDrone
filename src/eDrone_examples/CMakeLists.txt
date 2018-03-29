# cmake_minimum_required
# cmake 버전 정보

cmake_minimum_required(VERSION 2.8.3)

# project: 프로젝트 이름
project(eDrone_examples)

# find_package
# 캐킨 빌드를 할 때 필요한 패키지 목록 (설치되어 있지 않으면 에러 발생)
# find_package 명령을 통해 명시된 패키지들로부터 export 된 헤더 파일들의 include 경로, 라이브러리 파일의 경로가 캐킨 환경 변수에 추가됨
# ex. catkin_INCLUDE_DIRS, catkin_LIBRARIES
# find_package에서 명시된 패키지 목록은 package.xml 파일 내 DEPEND 목록에도 명시되어야 빌드 에러가 발생하지 않음

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	eDrone_msgs
	eDrone_lib
)


#find_library (
#eDroneLIB
#NAMES libeDrone_lib.so
#HINTS ~/devel/.private/eDrone_lib/lib
#HINTS ~
#REQUIRED 
#)


catkin_package(
	INCLUDE_DIRS inc
	CATKIN_DEPENDS roscpp rospy std_msgs 
	eDrone_msgs 
#	eDrone_lib
)

include_directories(
	# ~/devel/.private/eDrone_lib/lib
	~	
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
target_link_libraries(ex_check  
${eDroneLIB}
${catkin_LIBRARIES} )

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

add_executable(ex_survey_new src/ex_survey_new.cpp)

add_dependencies(ex_survey_new
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries (ex_survey_new ${catkin_LIBRARIES} )


add_executable(ex_spraying_mission src/ex_spraying_mission.cpp)

add_dependencies(ex_spraying_mission

 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
 )
target_link_libraries(ex_spraying_mission ${catkin_LIBRARIES} )


#add_executable(ex_lib src/ex_lib.cpp)

#add_dependencies(ex_lib

 #${${PROJECT_NAME}_EXPORTED_TARGETS}
 #${catkin_EXPORTED_TARGETS}
 #)
#target_link_libraries(ex_lib ${catkin_LIBRARIES} )




