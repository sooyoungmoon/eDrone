PRJ:projectName

---section_include
#include <eDrone_msgs/CheckHome.h>
---


---section_main_service_msg_variables
eDrone_msgs::CheckHome checkHome_cmd;
---

---section_main_variables
	double home_latitude;
	double home_longitude;
	double home_altitude;
---

---section_main_service_client
ros::ServiceClient checkHome_client =nh.serviceClient<eDrone_msgs::CheckHome>("srv_checkHome"); 
---


---section_main_service_call	
	    // 무인기 위치 확인

		 ROS_INFO("Send checkHome command ... \n");
		 ROS_INFO("Checking the position ... \n");

		 if (checkHome_client.call(checkHome_cmd))
		 {
			ROS_INFO ("CheckHome service was requested");

			home_latitude = checkHome_cmd.response.latitude;
			home_longitude = checkHome_cmd.response.longitude;
			home_altitude = checkHome_cmd.response.altitude;

			cout << "Home position: (" << home_latitude << ", " << home_longitude << ", " << home_altitude << ")" << endl; 
				
		}
			
			ROS_INFO("Home position was checked!");
---

