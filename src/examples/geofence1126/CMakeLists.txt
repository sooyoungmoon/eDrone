
# cmake_minimum_required
# cmake 버전 정보

cmake_minimum_required(VERSION 2.8.3)

# project: 프로젝트 이름
project(geofence1126)

# find_package
# 캐킨 빌드를 할 때 필요한 패키지 목록 (설치되어 있지 않으면 에러 발생)
# find_package 명령을 통해 명시된 패키지들로부터 export 된 헤더 파일들의 include 경로, 라이브러리 파일의 경로가 캐킨 환경 변수에 추가됨
# ex. catkin_INCLUDE_DIRS, catkin_LIBRARIES
# find_package에서 명시된 패키지 목록은 package.xml 파일 내 DEPEND 목록에도 명시되어야 빌드 에러가 발생하지 않음

find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	eDrone_msgs
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
	CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	eDrone_msgs
)
include_directories(
	# ~/devel/.private/eDrone_lib/lib
	~	
	inc
	${catkin_INCLUDE_DIRS}
	/usr/include
	/usr/include/c++/5
	/usr/include/c++/5/backward
	/usr/include/x86_64-linux-gnu
	/usr/include/x86_64-linux-gnu/c++/5
	/usr/include/x86_64-linux-gnu/5/include
	/usr/include/x86_64-linux-gnu/5/include-fixed
	/usr/local/include
)

#link_directories(~/catkin_ws/devel/lib)
add_executable(ex_geofence1126 src/ex_geofence1126.cpp)
add_dependencies (
        ex_geofence1126
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	
)
target_link_libraries(ex_geofence1126 ${catkin_LIBRARIES} )
