



PRJ:projectName

---section_include
#include <eDrone_msgs/MissionUpload.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_variables
	vector<mavros_msgs::Waypoint> waypoints;
---

---section_topic_callback_functions
---

---section_functions
	void print_waypoints (vector<mavros_msgs::Waypoint> waypoints) // 웨이포인트 정보
	{
		for (int i = 0; i < waypoints.size(); i++)
  		{
                	cout << "waypoint[" << i << "]: ";

                 	cout << endl ;

                	cout << waypoints[i] << endl;
        	}
	}
---

---section_main_init
---

---section_main_topic_msg_variables	
---

---section_main_service_msg_variables
	eDrone_msgs::MissionUpload missionUpload_cmd;
---

---section_main_variables
---

---section_main_topic_subscriber
---

---section_main_service_client
	 ros::ServiceClient missionUpload_client =nh.serviceClient<eDrone_msgs::MissionUpload>("srv_missionUpload");
---

---section_main_service_call	
	ROS_INFO("Send missionUpload command ... \n"); 


  	if (missionUpload_client.call (missionUpload_cmd) == true )
  	{ 
		ROS_INFO ("missionUpload command was sent to FC\n");		
	}
---


