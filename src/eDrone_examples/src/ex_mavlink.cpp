

// (2018.09.16) mavlink 메시지 송수신 예제 

/* header 파일 */
// ROS
#include <ros/ros.h>

// C++ std library
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// mavlink
#include <mavlink/v2.0/common/mavlink.h>


// 소켓 연결
#include <netinet/in.h>

// eDrone API
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>


/* namespace */

/* main */

int main (int argc, char** argv)
{

	 ROS_INFO ("ex_mavlink");

	 /* 변수 선언 */
	 char target_ip[100]; // FC ip 주소 
	 int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP); // 소켓 
  	 struct sockaddr_in gcAddr;
	 struct sockaddr_in locAddr;
	 mavlink_heartbeat_t hb;

	 /* FC 주소 설정 */
 	// Change the target ip if parameter was given
	strcpy(target_ip, "127.0.0.1");
	
	if (argc ==2)
	{	
		strcpy(target_ip, argv[1]);			
	}

 	/* udp addr. binding */
	


	return 0;
}
