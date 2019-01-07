
PRJ:projectName


---section_include
#include <eDrone_msgs/MissionClear.h>
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
	eDrone_msgs::MissionClear missionClear_cmd;
---

---section_main_topic_subscriber
---

---section_main_service_client
	 ros::ServiceClient missionClear_client =nh.serviceClient<eDrone_msgs::MissionClear>("srv_missionClear");
---

---section_main_service_call	
	ROS_INFO("Send missionClear command ... \n");
   	{
	
		if (missionClear_client.call(missionClear_cmd))
		{
			ROS_INFO ("missionClear command was sent to FC\n");
		}
   	}
---
