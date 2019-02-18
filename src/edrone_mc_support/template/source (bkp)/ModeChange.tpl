PRJ:projectName
MODECHANGE_MODE:MODECHANGE_MODE

---section_include
#include <eDrone_msgs/ModeChange.h>
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
	eDrone_msgs::ModeChange modeChange_cmd;
---

---section_main_topic_subscriber
---

---section_main_service_client
	ros::ServiceClient modeChange_client =nh.serviceClient<eDrone_msgs::ModeChange>("srv_modeChange");
---

---section_main_service_call	
   // ModeChange
	
	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}


	string modeChange_mode = ?[MODECHANGE_MODE];

	modeChange_cmd.request.modeChange_mode = modeChange_mode;

	ROS_INFO("Send ModeChange command ... \n");

	modeChange_client.call(modeChange_cmd); // 착륙 명령

	if (modeChange_cmd.response.value == true) // 서비스 호출 결과 확인 
	{
		ROS_INFO("ModeChange command was sent to FC\n");
	}


---


