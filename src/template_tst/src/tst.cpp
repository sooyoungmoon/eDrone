
/* include 문 */ 
#include <ros/ros.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>

#include "params.h"


/* 서비스 파일 선언부 */

eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;



/* main 함수*/

int main(int argc, char** argv)
{
   printf("==tst==\n");
   /* ros::init() 함수 호출 */ 
   ros::init(argc, argv, "tst");

  /*   ros::NodeHandle 객체 선언*/
   ros::NodeHandle nh;

 /* 파라미터 설정 */
  double altitude = ALTITUDE; // header 파일에서 디폴트 값을 읽어 와서 변수에 저장 
    

  //$(PARAMETER_TYPE) $(PARAMETER_VARIABLE) = $(PARAMETER_NAME); 
  nh.setParam("ALTITUDE", altitude);
  
  /* service client 선언 */ 
  ros::ServiceClient arming_client  =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");  
  ros::ServiceClient takeoff_client  =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");  
  ros::ServiceClient landing_client  =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");  

  /* service 요청 메시지 필드 설정 */

  //$(SERVICE_FILE_NAME)_cmd.$(PARAMETER_NAME) = $(PARAMETER_VALUE)
  
  takeoff_cmd.request.altitude = altitude;

  sleep(30);

  /* 서비스 호출 */

  // $(SERVICE_FILE_NAME)
  if (arming_client.call(arming_cmd))
	ROS_INFO("arming command was sent\n");


  if (takeoff_client.call(takeoff_cmd))
	ROS_INFO("takeoff command was sent\n");

  sleep(10);


  if (landing_client.call(landing_cmd))
	ROS_INFO("landing command was sent\n");
  
  return 0;
}

