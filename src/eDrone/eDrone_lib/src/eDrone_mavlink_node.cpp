/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 ****************************************************************************/
/*
This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.
 
 
 I compiled this program sucessfully on Ubuntu 10.04 with the following command
 
 gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c
 
 the rt library is needed for the clock_gettime on linux
 */

#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoInfo.h>
#include <eDrone_msgs/Arming.h> // 시동 서비스 헤더 파일 포함                                             
#include <eDrone_msgs/Takeoff.h> // 이륙 서비스 
#include <eDrone_msgs/Landing.h> // 착륙 서비스 	"
#include <eDrone_msgs/CheckState.h> // 무인기 상태 확인 서비스
#include <eDrone_msgs/CheckPosition.h> // 무인기 위치 확인 서비스
#include <eDrone_msgs/CheckHome.h> // home 위치 확인 서비스
#include <eDrone_msgs/Heartbeat.h> // FC로부터 수신된 heartbeat 메시지 publish 

// mavlink 통신 관련 
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
//#include <mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
//


using namespace std;
using namespace Mission_API; 
using namespace mavros_msgs;
using namespace sensor_msgs;


uint64_t microsSinceEpoch();

//// 메시지 변수 선언

// (무인기 상태 정보 수신 목적)
mavros_msgs::State current_state; // 무인기 상태 정보

// (무인기 위치 확인 목적)

// (홈 위치 획득 목적)
//mavros_msgs::HomePosition home_position; // home position

// Heartbeat 메시지 
eDrone_msgs::Heartbeat heartbeat_msg; 

// publisher 선언

ros::Publisher mavlink_pub;

// subscriber 선언

ros::Subscriber mavlink_sub;

// 서비스 서버 선언

ros::ServiceServer chkState_srv_server;
ros::ServiceServer chkPosition_srv_server;
ros::ServiceServer chkHome_srv_server;

// home position

 double HOME_LAT ;
 double HOME_LON;
 double HOME_ALT;

// mavlink 통신에 필요한 변수
char target_ip[100];
	
float position[6] = {};
int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
struct sockaddr_in gcAddr; 
struct sockaddr_in locAddr;
//struct sockaddr_in fromAddr;
uint8_t buf[BUFFER_LENGTH];
ssize_t recsize;
socklen_t fromlen;
int bytes_sent;
mavlink_message_t msg;
uint16_t len;
int i = 0;
//int success = 0;
unsigned int temp = 0;

//





// callback 함수 (서비스 제공) 정의





int main(int argc, char** argv)

{

	ros::init(argc, argv, "eDrone_mavlink_node");
 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 

	// publisher 초기화
	mavlink_pub = nh.advertise<eDrone_msgs::Heartbeat>("eDrone_msgs/heartbeat", 10);

	// subscriber 초기화
	
	// 서비스 서버 선언	

	
	// target IP 설정 (GCS)
		strcpy(target_ip, "127.0.0.1");
		
		if (argc == 2)
  	        {
			strcpy(target_ip, argv[1]);
    		}

		
		// local address 설정

		memset(&locAddr, 0, sizeof(locAddr));
		locAddr.sin_family = AF_INET;
		locAddr.sin_addr.s_addr = INADDR_ANY;
		locAddr.sin_port = htons(14551);
	
		/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
		if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
	        {
			perror("error bind failed");
			close(sock);
			exit(EXIT_FAILURE);
    		}
		
		/* Attempt to make it non blocking */
		#if (defined __QNX__) | (defined __QNXNTO__)
		if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
		#else
		if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
		#endif

  		{
			fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
			close(sock);
			exit(EXIT_FAILURE);
    		}
	
		// gcs 주소 설정 
		memset(&gcAddr, 0, sizeof(gcAddr));
		gcAddr.sin_family = AF_INET;
		gcAddr.sin_addr.s_addr = inet_addr(target_ip);
		gcAddr.sin_port = htons(14550);


	while ( ros::ok() )
	{	

		/* MAVLink 메시지 송수신 */

		/*Send Heartbeat */
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		/* Send Status */
		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
		
		/* Send Local Position */
		mavlink_msg_local_position_ned_pack(1, 200, &msg, microsSinceEpoch(), 
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		/* Send attitude */
		mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0)
      		{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			
			printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				}
			}
			printf("\n");
		}
		memset(buf, 0, BUFFER_LENGTH);
		sleep(1); // Sleep one second    

		//	 

		// heartbeat 메시지 필드 설정 
		/*
		heartbeat_msg.custom_mode = received_msg.custom_mode
		...
		*/
		//
		mavlink_pub.publish(heartbeat_msg); // 수신된 heartbeat 메시지 publish		
	
		ros::spinOnce();
      		rate.sleep();				

/*
		cout << "eDrone_monitor_node: " ;
		printf(" home position: (%lf, %lf, %lf) \n", home.latitude, home.longitude, home.altitude);	
*/
	}


	return 0;
}

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}
#else
uint64_t microsSinceEpoch()
{
	
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}
#endif

