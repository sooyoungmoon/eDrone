
PRJ:projectName
RADIUS:RADIUS

---section_include
#include <eDrone_msgs/GeofenceSet.h>
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
	eDrone_msgs::GeofenceSet geofenceSet_cmd;
---

---section_main_variables
	double geofence_radius= 0;
---

---section_main_topic_subscriber
---

---section_main_service_client
	 ros::ServiceClient geofenceSet_client =nh.serviceClient<eDrone_msgs::GeofenceSet>("srv_geofenceSet");
---

---section_main_service_call	
	geofence_radius = ?[RADIUS];
	geofenceSet_cmd.request.radius = geofence_radius;

	if (geofenceSet_client.call (geofenceSet_cmd) == true)
	{
		ROS_INFO ("ex_geofence: GeofenceSet service was called");
		
		if (geofenceSet_cmd.response.value == true)
		{
			ROS_INFO ("ex_geofence: Geofence was set (radius= %lf) \n", geofence_radius);
		}

	}	


---


