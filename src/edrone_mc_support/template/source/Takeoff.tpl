PRJ:projectName
TAKEOFF_ALTITUDE:TAKEOFF_ALTITUDE

---section_include
#include <eDrone_msgs/Takeoff.h>
---

---section_main_service_msg_variables
	eDrone_msgs::Takeoff takeoff_cmd;
---

---section_main_service_client
	ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
---

// ROS Service 호출 
---section_main_service_call	
  // Takeoff
	
	double takeoff_altitude = 0;

	takeoff_altitude = ?[TAKEOFF_ALTITUDE];
	// 1) 상수에 의한 초기화: takeoff_altitude = TAKEOFF_ALTITUDE;
	// 2) 명령줄 인자에 의한 초기화: takeoff_altitude = atof (argv[1]); 

	ROS_INFO("takeoff_altitude: %lf", takeoff_altitude);

	ROS_INFO("Send takeoff command ... \n");
	takeoff_cmd.request.takeoff_altitude = takeoff_altitude; // 서비스 파라미터 설정
	takeoff_client.call(takeoff_cmd); // 서비스 호출

	if (takeoff_cmd.response.value == true) // 서비스 호출 결과 확인 
	{
		ROS_INFO("Takeoff command was sent to FC\n");
	}
		
	sleep(10);
---
