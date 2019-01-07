
PRJ:projectName
RADIUS:RADIUS

---section_include
#include <eDrone_msgs/GeofenceReset.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_variables
---

---section_topic_callback_functions
---

---section_functions
	
---

---section_main_init
---

---section_main_topic_msg_variables	
---

---section_main_service_msg_variables
	eDrone_msgs::GeofenceReset geofenceReset_cmd;
---

---section_main_variables	
---

---section_main_topic_subscriber
---

---section_main_service_client
	 ros::ServiceClient geofenceReset_client =nh.serviceClient<eDrone_msgs::GeofenceReset>("srv_geofenceReset");
---

---section_main_service_call	
	cout <<"\nGeofenceReset 서비스 요청 " << endl;	
	
	if (geofenceReset_client.call(geofenceReset_cmd) == true)
	{
		
		if (geofenceReset_cmd.response.value == true)
		{
			ROS_INFO ("GeofenceReset command success!");
		}
		else
		{
			ROS_INFO ("GeofenceReset command fail!");

		}
	}


---


