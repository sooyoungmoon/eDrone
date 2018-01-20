cmake_minimum_required(VERSION 2.8.3)
project(template_tst)
find_package(catkin REQUIRED COMPONENTS 

  rospy
  roscpp
  eDrone_msgs
  std_msgs
  message_generation
  mavros_msgs

)
# catkin_python_setup()
# add_message_files
# ROS 노드 간 통신에 사용되는 토픽 메시지 파일 목록 - 현재 패키지 내 msg 디렉토리에 해당 메시지 파일이 생성되어 있어야 함

# catkin_package 
# 캐킨 빌드에서 cmake 파일과 config 파일 생성에 사용되는 명령어 (매크로 함수)
 #  패키지 config. 파일 (.pc)은 ~/catkin_ws/devel/.private/mavros/lib/pkgconfig/ 디렉토리에 생성됨
# cmake 파일은 ~/catkin_ws/devel/.private/mavros_msgs/share/mavros_msgs/cmake/에 생성됨 


# INCLUDE_DIRS: 헤더 파일이 위치한 디렉토리 지정 (패키지 디렉토리에 대한 상대 경로)
# LIBRARIES: 다른 패키지에서 생성된 라이브러리 사용 (해당 패키지 이름 지정)
# CATKIN_DEPENDS: 의존 패키지 (ROS package)목록→ .pc 파일 내 ‘Requires:’ 영역에 복사됨 
# DEPENDS: ROS 패키지가 아닌 의존 패키지 목록
# CATKIN_DEPENDS 또는 DEPENDS 에 명시된 패키지는 package.xml 내 ‘<depend>’ 태그 내에 명시되어야 하며 그렇지 않을 경우 빌드 과정에서 에러 발생

catkin_package(
  INCLUDE_DIRS inc
  CATKIN_DEPENDS 
  rospy
  roscpp
  eDrone_msgs
  std_msgs
  message_generation
  mavros_msgs
)


# Include Paths and Library Paths

# include_directories: 인클루드 대상 헤더 파일 경로 지정 (include_directories(<dir1>, <dir2>, ..., <dirN>))

#link_directories:  타겟 빌드에 필요한 라이브러리 경로 지정

#link_directories(<dir1>, <dir2>, ..., <dirN>)
# find_package로 지정된 패키지들의 라이브러리 경로는 자동으로 환경 변수에 추가됨
# link_directories 대신 target_link_libraries 사용 권장 (ROS wiki)
include_directories(
 inc
 ${catkin_INCLUDE_DIRS}
)

# add_library / add_executable / target_link_libraries
# add_library: 생성될 라이브러리 파일 명시
# add_executable: 생성될 실행 파일 명시
# add_dependencies: 패키지 A가 자기 자신 또는 다른 패키지에서 정의된 메시지 또는 서비스를 사용할 경우,
# 실행 파일을 생성하기 전에 먼저 해당 메시지/서비스를 먼저 생성하도록 지정할 수 있음
# 현재 패키지에서 정의된 메시지/서비스 사용 시, 의존성 목록에 
# ${${PROJECT_NAME}_EXPORTED_TARGETS}  추가
# 다른 패키지에서 정의된 메시지/서비스 사용 시, 의존성 목록에 $catkin_EXPORTED_TARGETS 추가

add_executable ( 
   tst 
   src/tst.cpp
    
)

add_dependencies ( 
   tst
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
   tst
   ${catkin_LIBRARIES}
)

add_executable ( 
   mission 
   src/mission.cpp
    
)

add_dependencies ( 
   mission
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
   mission
   ${catkin_LIBRARIES}
)

