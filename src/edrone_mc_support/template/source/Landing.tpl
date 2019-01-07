PRJ:projectName



---section_include
#include <eDrone_msgs/Landing.h>
---

---section_namespace
---

---section_pointer_variables
eDrone_msgs::Landing landing_cmd;
---

---section_topic_callback_functions
---

---section_main_init
---
	

---section_main_topic_msg_variables
---  

---section_main_service_msg_variables
---

---section_main_topic_subscriber
---

---section_main_service_client
ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
---

---section_main_service_call	
   // Landing
	
	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}

	ROS_INFO("Send landing command ... \n");

	landing_client.call(landing_cmd); // 착륙 명령

	if (landing_cmd.response.value == true) // 서비스 호출 결과 확인 
	{
		ROS_INFO("Landing command was sent to FC\n");
	}


---


