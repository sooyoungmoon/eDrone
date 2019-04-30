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
	mavros
	eDrone_msgs
	libmavconn
	
)

# 요구되는 패키지 명시 

## Find GeographicLib
find_package(GeographicLib)

# topic 선언 (eDrone_msgs 패키지에서 설정)

# service 선언 (eDrone_msgs 패키지에서 설정)

# 메시지 생성 (eDrone_msgs 패키지에서 설정))

###################################
## catkin specific configuration ##
###################################

catkin_package(

INCLUDE_DIRS inc
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

add_executable (eDrone_monitor_node
src/eDrone_monitor_node.cpp 
src/Vehicle.cpp 
src/GeoInfo.cpp 
src/GeoUtils.cpp
)

target_link_libraries(eDrone_monitor_node 
${catkin_LIBRARIES} 
${GeographicLib_LIBRARIES}
)

add_executable (eDrone_autoflight_node 
src/eDrone_autoflight_node.cpp 
src/GeoUtils.cpp)

target_link_libraries(eDrone_autoflight_node 
${catkin_LIBRARIES} 
${GeographicLib_LIBRARIES})

add_executable (eDrone_control_node 
src/eDrone_control_node.cpp 
src/GeoUtils.cpp)

target_link_libraries(eDrone_control_node 
${catkin_LIBRARIES}
 ${GeographicLib_LIBRARIES}
)

# (2018.07.16) 신규 노드 추가 (eDrone_safety_node: Geofence, NoflyZone API 구현) 
add_executable (eDrone_safety_node
src/eDrone_safety_node.cpp
src/GeoUtils.cpp
)

target_link_libraries (eDrone_safety_node
 ${catkin_LIBRARIES}
 ${GeographicLib_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation


install(TARGETS eDrone_monitor_node eDrone_autoflight_node eDrone_control_node eDrone_safety_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# installing header files
install(DIRECTORY inc/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# installing roslaunch files or other resources
install (DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
