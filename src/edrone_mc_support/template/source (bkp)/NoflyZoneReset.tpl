

PRJ:projectName

---section_include
#include <eDrone_msgs/NoflyZoneReset.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_topic_callback_functions
---


---section_main_service_msg_variables
	eDrone_msgs::NoflyZoneReset noflyZoneReset_cmd;
---

---section_main_service_client
	ros::ServiceClient noflyZoneReset_Client<eDrone_msgs::NoflyZoneReset>("srv_noflyZoneReset");
---

---section_main_service_call	
	{
		if (noflyZoneReset_client.call (noflyZoneReset_cmd) == true)
		{
			ROS_INFO("NoflyZoneReset command was sent to FC");

			if (noflyZoneReset_cmd.response.value == true)
			{
				ROS_INFO("noflyZone was removed");
			}
		}	

	}
---
