PRJ:projectName
ORBIT_REF_SYSTEM:ORBIT_REF_SYSTEM
ORBIT_CENTER:ORBIT_CENTER
ORBIT_RADIUS:ORBIT_RADIUS
ORBIT_REQ_CNT:ORBIT_REQ_CNT

---section_include
#include <eDrone_msgs/Orbit.h>
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
	eDrone_msgs::Orbit orbit_cmd;
---

---section_main_topic_subscriber
---

---section_main_service_client
	ros::ServiceClient orbit_client =nh.serviceClient<eDrone_msgs::Orbit>("srv_orbit");
---

---section_main_service_call	
   // Orbit
	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;      
	}
	
	string orbit_ref_system = ?[ORBIT_REF_SYSTEM];
	
	Target orbit_center;
	string orbit_center_str = ?[ORBIT_CENTER];
	orbit_cmd.request.orbit_ref_system = orbit_ref_system;
	
	// Target이 구조체이므로 orbit_center을 초기화 하는데 필요한 문자열 변수 (orbit_center_str)가 선언되었다는 가정 하에 아래 코드 생성  	

	// 1) 상수에 의한 초기화의 경우 	   
	  // (params.h에 선언) string ORBIT_CENTER = <초기값> (예: "30,30,10");	
          // orbit_center_str = ORBIT_CENTER;
	
	  // 2) 명령줄 인자에 의한 초기화의 경우 
	  // orbit_center_str = argv[i]; 

	// 1), 2) 공통  
	{
		// 도구에서 자동 생성 요 (시작)
			int numCnt = 0;
			vector<string>  strVector;

			string token;
			size_t delimiter_pos = 0;
			delimiter_pos = orbit_center_str.find(",");

			while (delimiter_pos != string::npos)
			{
				numCnt++;
				token = orbit_center_str.substr(0, delimiter_pos);
				orbit_center_str = orbit_center_str.substr (delimiter_pos+1);
				cout << "token: " << token << endl;
				strVector.push_back(token);
				cout << "orbit_center_str: " << orbit_center_str << endl;
				delimiter_pos = orbit_center_str.find(",");
			}

			token = orbit_center_str.substr(0, delimiter_pos);
			strVector.push_back(token);
			numCnt++;
			cout << "token: " << token << endl;
			cout << "numCnt: " << numCnt << endl;

		// 도구에서 자동 생성 요 (종료)

			string x_lat_str = strVector[0];
			orbit_center.x_lat = atof (x_lat_str.c_str());

			string y_long_str = strVector[1];
			orbit_center.y_long = atof (y_long_str.c_str());

			string z_alt_str = strVector[2];
			orbit_center.z_alt = atof (z_alt_str.c_str());

			orbit_cmd.request.orbit_center = orbit_center;
	} 
		
	double orbit_radius = ?[ORBIT_RADIUS];
	orbit_cmd.request.orbit_radius = orbit_radius;

	int orbit_req_cnt = ?[ORBIT_REQ_CNT];
	orbit_cmd.request.orbit_req_cnt = orbit_req_cnt;

	ROS_INFO("Send Orbit command ... \n");

	orbit_client.call(orbit_cmd); // 착륙 명령

	if (orbit_cmd.response.value == true) // 서비스 호출 결과 확인 
	{
		ROS_INFO("Orbit command was called\n");

		while (cur_phase.phase.compare ("READY") ==0)
		{
				ros::spinOnce();
				rate.sleep();
				cout << "cur_phase: " << cur_phase_ptr->phase << endl;
		}
	}


---


