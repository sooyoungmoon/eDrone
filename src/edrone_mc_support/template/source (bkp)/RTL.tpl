PRJ:projectName

---section_include
#include <eDrone_msgs/RTL.h>
---

---section_main_service_msg_variables
	eDrone_msgs::RTL rtl_cmd;
---


---section_main_service_client
	ros::ServiceClient rtl_client = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");
---

---section_main_service_call	
	// RTL

	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}

	rtl_client.call(rtl_cmd); // rtl service 호출 (복귀)
	
	if (rtl_cmd.response.value == true)
	{
		ROS_INFO("RTL command was sent\n");
        }

---
