
PRJ:projectName

---section_include
#include <mavros_msgs/Waypoint.h>
#include <eDrone_msgs/MissionDownload.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_variables
	vector<mavros_msgs::Waypoint> waypoints;
---

---section_functions

---

---section_topic_callback_functions
---



---section_main_init
---

---section_main_topic_msg_variables	
---

---section_main_service_msg_variables
	eDrone_msgs::MissionDownload missionDownload_cmd;
---

---section_main_variables
	vector<mavros_msgs::Waypoint> waypoints;
---

---section_main_topic_subscriber
---

---section_main_service_client
	 ros::ServiceClient missionDownload_client =nh.serviceClient<eDrone_msgs::MissionDownload>("srv_missionDownload");
---

---section_main_service_call	
	ROS_INFO("Send missionDownload command ... \n"); 


  	if (missionDownload_client.call (missionDownload_cmd) == true )
  	{ 
		ROS_INFO ("missionDownload command was sent\n");
		waypoints = missionDownload_cmd.response.waypoints;
		print_waypoints(waypoints);
	}
---


