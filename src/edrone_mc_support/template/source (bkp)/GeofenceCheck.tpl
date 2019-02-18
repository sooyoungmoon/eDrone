
PRJ:projectName
RADIUS:RADIUS
GEOFENCE_REF_SYSTEM:GEOFENCE_REF_SYSTEM
GEOFENCE_ARG1:GEOFENCE_ARG1
GEOFENCE_ARG2:GEOFENCE_ARG2
GEOFENCE_ARG3:GEOFENCE_ARG3

---section_include
#include <eDrone_msgs/GeofenceCheck.h>
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
	eDrone_msgs::GeofenceCheck geofenceCheck_cmd;
---

---section_main_variables
	bool geofence_violation = false;
---

---section_main_topic_subscriber
---

---section_main_service_client
	 ros::ServiceClient geofenceCheck_client =nh.serviceClient<eDrone_msgs::GeofenceCheck>("srv_geofenceCheck");
---

---section_main_service_call	

	string geofence_ref_system = ?[GEOFENCE_REF_SYSTEM];	
	geofenceCheck_cmd.request.radius = geofence_radius;

	double geofence_arg1 = ?[GEOFENCE_ARG1];
	geofenceCheck_cmd.request.geofence_arg1 = geofence_arg1;

	double geofence_arg2 = ?[GEOFENCE_ARG2];
	geofenceCheck_cmd.request.geofence_arg2 = geofence_arg2;

	double geofence_arg3 = ?[GEOFENCE_ARG3];
	geofenceCheck_cmd.request.geofence_arg3 = geofence_arg3;

	if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
	{
		ROS_INFO ("GeofenceCheck service was called");
		
		if (geofenceCheck_cmd.response.value == true)
		{
			if (geofenceCheck_cmd.response.violation == true)
			{
				ROS_INFO ("Geofence violation!\n");
			}
		}

	}	


---


