PRJ:projectName

---section_include
#include <eDrone_msgs/Arming.h>
---


---section_main_service_msg_variables
	eDrone_msgs::Arming arming_cmd;
---

---section_main_service_client
	ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
---

---section_main_service_call	
	// Arming

		ROS_INFO("Send arming command ... \n"); 
		arming_client.call(arming_cmd);
		
		if (arming_cmd.response.value == true)
		{
			ROS_INFO("Arming command was sent to FC");
		}
---
