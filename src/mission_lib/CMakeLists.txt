cmake_minimum_required(VERSION 2.8.3)
project(mission_lib)


# 요구되는 패키지 목록 (캐킨 패키지) - mission_lib_msgs 패키지 포함
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs	
	geometry_msgs
	sensor_msgs
	geographic_msgs	
	mavros_msgs
	mission_lib_msgs
	libmavconn
	
)

# 요구되는 패키지 명시 

## Find GeographicLib
find_package(GeographicLib)

# topic 선언 (mission_lib_msgs 패키지에서 설정)
#add_message_files(FILES Target.msg)

# service 선언 (mission_lib_msgs 패키지에서 설정)

#add_service_files (FILES Arming.srv Takeoff.srv Landing.srv CheckState.srv CheckPosition.srv CheckHome.srv ModeChange.srv RTL.srv MissionAddItem.srv MissionUpload.srv MissionDownload.srv MissionClear.srv Goto.srv Geofence.srv NoflyZone.srv CheckNFZone.srv)

# 메시지 생성 (mission_lib_msgs 패키지에서 설정))
#generate_messages(DEPENDENCIES std_msgs mavros_msgs geometry_msgs geographic_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(

INCLUDE_DIRS include
LIBRARIES mission_lib
CATKIN_DEPENDS sensor_msgs geographic_msgs std_msgs libmavconn message_runtime mavros_msgs
DEPENDS GeographicLib

)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${mavlink_INCLUDE_DIRS}
	${GeographicLib_INCLUDE_DIRS}	
)

# 여러 개의 노드를 하나의 라이브러리로 생성하려고 할 경우 에러 발생 (multiple definitions of main fuctions)
add_library(mission_lib src/GeoInfo.cpp src/GeoUtils.cpp src/Vehicle.cpp)



#add_executable (mission_lib_node src/mission_lib_node.cpp src/Vehicle.cpp)

#target_link_libraries(mission_lib_node ${catkin_LIBRARIES})
#add_dependencies(mission_lib_node mission_lib_generate_messages_cpp)

add_executable (mission_lib_common 
src/mission_lib_common.cpp 
src/Vehicle.cpp 
src/GeoInfo.cpp 
src/GeoUtils.cpp
)

target_link_libraries(mission_lib_common 
 mission_lib
${catkin_LIBRARIES} 
${GeographicLib_LIBRARIES}
)

add_dependencies(mission_lib_common 
mission_lib_generate_messages_cpp)

add_executable (mission_lib_mission 
src/mission_lib_mission.cpp 
src/GeoUtils.cpp)

target_link_libraries(mission_lib_mission 
 mission_lib
${catkin_LIBRARIES} 
${GeographicLib_LIBRARIES})

add_dependencies(mission_lib_mission mission_lib_generate_messages_cpp)

add_executable (mission_lib_control 
src/mission_lib_control.cpp 
src/GeoUtils.cpp)

target_link_libraries(mission_lib_control 
 mission_lib
${catkin_LIBRARIES}
 ${GeographicLib_LIBRARIES}
)

add_dependencies(mission_lib_control
mission_lib_generate_messages_cpp)


add_executable (mission_lib_agrc 
src/mission_lib_agrc.cpp 
src/GeoUtils.cpp)

target_link_libraries(mission_lib_agrc 
 mission_lib
${catkin_LIBRARIES}
 ${GeographicLib_LIBRARIES}
)

add_dependencies(mission_lib_agrc
mission_lib_generate_messages_cpp)


#add_executable (mission_lib_agrc src/mission_lib_agrc.cpp src/GeoUtils.cpp)
#target_link_libraries(mission_lib_agrc ${catkin_LIBRARIES})
#add_dependencies(mission_lib_agrc mission_lib_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


## Mark other files for installation (e.g. launch and bag files, etc.)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


#############
## Testing ##
#############

# ToDo





