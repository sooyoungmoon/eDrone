PRJ:projectName
ORBIT_REF_SYSTEM:ORBIT_REF_SYSTEM
ORBIT_X_LAT:ORBIT_X_LAT
ORBIT_Y_LONG:ORBIT_Y_LONG
ORBIT_Z_ALT:ORBIT_Z_ALT
ORBIT_RADIUS:ORBIT_RADIUS
ORBIT_REQ_CNT:ORBIT_REQ_CNT

---section_include
#include <eDrone_msgs/Orbit.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_topic_callback_functions
---

---section_main_init
---	

---section_main_topic_msg_variables
---  

---section_main_service_msg_variables
	eDrone_msgs::Orbit orbit_cmd;
---

---section_main_topic_subscriber
---

---section_main_service_client
	ros::ServiceClient orbit_client =nh.serviceClient<eDrone_msgs::Orbit>("srv_orbit");
---

---section_main_service_call	
   // Orbit
	
	string orbit_ref_system = ?[ORBIT_REF_SYSTEM];

	orbit_cmd.request.orbit_ref_system = orbit_ref_system;
	
	double orbit_x_lat = ?[ORBIT_X_LAT];
	orbit_cmd.request.orbit_x_lat = orbit_x_lat;

	double orbit_y_long = ?[ORBIT_Y_LONG];
	orbit_cmd.request.orbit_y_long = orbit_y_long;

	double orbit_z_alt = ?[ORBIT_Z_ALT];
	orbit_cmd.request.orbit_z_alt = orbit_z_alt;	

	double float64 orbit_radius = ?[ORBIT_RADIUS];
	orbit_cmd.request.orbit_radius = orbit_radius;

	int orbit_req_cnt = ?[ORBIT_REQ_CNT];
	orbit_cmd.request.orbit_req_cnt = orbit_req_cnt;

	ROS_INFO("Send Orbit command ... \n");

	orbit_client.call(orbit_cmd); // 착륙 명령

	if (orbit_cmd.response.value == true) // 서비스 호출 결과 확인 
	{
		ROS_INFO("Orbit command was sent to FC\n");
	}


---


