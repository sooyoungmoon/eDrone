cmake_minimum_required(VERSION 2.8.3)
project(eDrone_lib)


# 요구되는 패키지 목록 (캐킨 패키지) - eDrone_msgs 패키지 포함
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs	
	geometry_msgs
	sensor_msgs
	geographic_msgs	
	mavros_msgs
	eDrone_msgs
	libmavconn
	
)

# 요구되는 패키지 명시 

## Find GeographicLib
find_package(GeographicLib)

# topic 선언 (eDrone_msgs 패키지에서 설정)
#add_message_files(FILES Target.msg)

# service 선언 (eDrone_msgs 패키지에서 설정)

#add_service_files (FILES Arming.srv Takeoff.srv Landing.srv CheckState.srv CheckPosition.srv CheckHome.srv ModeChange.srv RTL.srv MissionAddItem.srv MissionUpload.srv MissionDownload.srv MissionClear.srv Goto.srv Geofence.srv NoflyZone.srv CheckNFZone.srv)

# 메시지 생성 (eDrone_msgs 패키지에서 설정))
#generate_messages(DEPENDENCIES std_msgs mavros_msgs geometry_msgs geographic_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(

INCLUDE_DIRS inc
LIBRARIES eDrone_lib
CATKIN_DEPENDS sensor_msgs geographic_msgs std_msgs libmavconn message_runtime mavros_msgs eDrone_msgs
DEPENDS GeographicLib

)

###########
## Build ##
###########

include_directories(
	inc
	${catkin_INCLUDE_DIRS}
	${mavlink_INCLUDE_DIRS}
	${GeographicLib_INCLUDE_DIRS}	
)

# 여러 개의 노드를 하나의 라이브러리로 생성하려고 할 경우 에러 발생 (multiple definitions of main fuctions)
add_library(eDrone_lib src/GeoInfo.cpp src/GeoUtils.cpp src/Vehicle.cpp)


add_executable (eDrone_monitor_node
src/eDrone_monitor_node.cpp 
src/Vehicle.cpp 
src/GeoInfo.cpp 
src/GeoUtils.cpp
)

target_link_libraries(eDrone_monitor_node 
 eDrone_lib
${catkin_LIBRARIES} 
${GeographicLib_LIBRARIES}
)

#add_dependencies(eDrone_monitor_node eDrone_lib_generate_messages_cpp)

add_executable (eDrone_utility_node 
src/eDrone_utility_node.cpp 
src/GeoUtils.cpp)

target_link_libraries(eDrone_utility_node 
 eDrone_lib
${catkin_LIBRARIES} 
${GeographicLib_LIBRARIES})

#add_dependencies(eDrone_utility_node eDrone_lib_generate_messages_cpp)

add_executable (eDrone_control_node 
src/eDrone_control_node.cpp 
src/GeoUtils.cpp)

target_link_libraries(eDrone_control_node 
 eDrone_lib
${catkin_LIBRARIES}
 ${GeographicLib_LIBRARIES}
)

#add_dependencies(eDrone_control_node eDrone_lib_generate_messages_cpp)


add_executable (eDrone_application_node 
src/eDrone_application_node.cpp 
src/GeoUtils.cpp)

target_link_libraries(eDrone_application_node 
 eDrone_lib
${catkin_LIBRARIES}
 ${GeographicLib_LIBRARIES}
)

#add_dependencies(eDrone_application_node eDrone_lib_generate_messages_cpp)


#add_executable (eDrone_application_node src/eDrone_application_node.cpp src/GeoUtils.cpp)
#target_link_libraries(eDrone_application_node ${catkin_LIBRARIES})
#add_dependencies(eDrone_application_node eDrone_lib_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation


install(TARGETS eDrone_application_node
#install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)



## Mark other files for installation (e.g. launch and bag files, etc.)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)




#############
## Testing ##
#############

# ToDo





