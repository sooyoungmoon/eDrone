cmake_minimum_required(VERSION 2.8.3)
project(mission_lib_msgs)


# find_package: 패키지 빌드에 필요한 의존 패키지 목록 - 미 설치 시 에러
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	geometry_msgs
	sensor_msgs
	geographic_msgs	
	mavros_msgs	
	#mission_lib
)


# topic 선언
# 사용되는 메시지 파일 목록: 현재 패키지 내 msg 디렉토리에 위치해야 함-->
add_message_files(
DIRECTORY msg
FILES 
Target.msg
) 

# service 선언
# 사용되는 서비스 파일 목록: 현재 패키지 내 srv 디렉토리에 위치해야 함-->
add_service_files (
DIRECTORY srv
FILES 
Arming.srv 
Takeoff.srv 
Landing.srv 
CheckState.srv 
CheckPosition.srv 
CheckHome.srv 
ModeChange.srv 
RTL.srv 
MissionAddItem.srv 
MissionUpload.srv 
MissionDownload.srv
MissionClear.srv
Goto.srv
Geofence.srv
NoflyZone.srv
CheckNFZone.srv
Survey.srv
)

# 메시지 생성을 위한 매크로 함수
generate_messages(DEPENDENCIES std_msgs mavros_msgs geometry_msgs geographic_msgs)


# 캐킨 빌드에서 cmake 파일과 config 파일 생성에 사용되는 명령어 (매크로 함수)
catkin_package(
INCLUDE_DIRS include
#LIBRARIES mission_lib
CATKIN_DEPENDS geometry_msgs sensor_msgs geographic_msgs message_runtime std_msgs
)

# catkin_INCLUDE_DIRS 환경 변수에 포함된 경로들을 include path에 포함 (find_package 명령으로 지정된 패키지들의 헤더 파일 포함)

include_directories( 
	include
	${catkin_INCLUDE_DIRS}       
)

install (
DIRECTORY include
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)





