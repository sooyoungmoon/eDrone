PRJ:projectName

---section_include
#include <eDrone_msgs/CheckState.h>
---


---section_main_service_msg_variables
	eDrone_msgs::CheckState checkState_cmd;
---



---section_main_service_client
	ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
---

---section_main_service_call	

	    // 연결 상태 확인

		sleep(10); // (수정)
		ROS_INFO("Send checkState command ... \n");
		ROS_INFO("Checking the connection ... \n");
		
		if (checkState_client.call(checkState_cmd))
		{
			ROS_INFO ("CheckState service was requested");
			
			while (checkState_cmd.response.connected == false)
			{
				if (checkState_client.call(checkState_cmd))
				{
					ROS_INFO ("Checking state...");
				}

				ros::spinOnce();
				rate.sleep();
			}

			ROS_INFO("UAV connection established!");

			if (checkState_cmd.response.connected == true)
			{
				cout << "UAV connected: " << endl;
			}
			
			if (checkState_cmd.response.armed == true )
			{
				cout << "UAV armed: " << endl;
			}

			cout << "flight mode: " << checkState_cmd.response.mode << endl;

			cout << "remaining battery(%): " << checkState_cmd.response.battery_remain << endl;
		}

---
