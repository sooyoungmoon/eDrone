

PRJ:projectName
NFCHECK_REF_SYSTEM:NFCHECK_REF_SYSTEM
ARG1:ARG1
ARG2:ARG2
ARG3:ARG3
SRC_ARG1:SRC_ARG1
SRC_ARG1:SRC_ARG2
SRC_ARG1:SRC_ARG3
DST_ARG1:DST_ARG1
DST_ARG1:DST_ARG2
DST_ARG1:DST_ARG3

---section_include
#include <eDrone_msgs/NoflyZoneCheck.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_topic_callback_functions
---


---section_main_service_msg_variables
	eDrone_msgs::NoflyZoneCheck noflyZoneCheck_cmd;
---

---section_main_service_client
	ros::ServiceClient noflyZoneCheck_Client<eDrone_msgs::NoflyZoneCheck>("srv_noflyZoneCheck");
---

---section_main_service_call	
	{
		noflyZoneCheck_cmd.request.NFCheck_ref_system = ?[NFCHECK_REF_SYSTEM];
		noflyZoneCheck_cmd.request.arg1 = ?[ARG1];
		noflyZoneCheck_cmd.request.arg2 = ?[ARG2];
		noflyZoneCheck_cmd.request.arg3 = ?[ARG3];

		noflyZoneCheck_cmd.request.src_arg1 = ?[SRC_ARG1];
		noflyZoneCheck_cmd.request.src_arg2 = ?[SRC_ARG2];
		noflyZoneCheck_cmd.request.src_arg3 = ?[SRC_ARG3];

		noflyZoneCheck_cmd.request.dst_arg1 = ?[DST_ARG1];
		noflyZoneCheck_cmd.request.dst_arg2 = ?[DST_ARG2];
		noflyZoneCheck_cmd.request.dst_arg3 = ?[DST_ARG3];
		
		
	}
---
