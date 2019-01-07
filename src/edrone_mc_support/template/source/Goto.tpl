PRJ:projectName
GOTO_REF_SYSTEM:GOTO_REF_SYSTEM
GOTO_POINT:GOTO_POINT

---section_include
#include <eDrone_msgs/Goto.h>
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
	eDrone_msgs::Goto goto_cmd;
---

---section_main_topic_subscriber
---

---section_main_service_client
	ros::ServiceClient goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
---

---section_main_service_call	
	// Goto

	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}

	string goto_ref_system = ?[GOTO_REF_SYSTEM];
	goto_cmd.request.goto_ref_system = goto_ref_system;
	
	Target goto_point; // template에 포함	
        string goto_point_str = ?[GOTO_POINT]; // template에 포함
	
	
	// Target이 구조체이므로 goto_point을 초기화 하는데 필요한 문자열 변수 (goto_point_str)가 선언되었다는 가정 하에 아래 코드 생성  	

	// 1) 상수에 의한 초기화의 경우 	   
	  // (params.h에 선언) string GOTO_POINT_STR = <초기값> (예: "30,30,10");	
          // goto_point_str = GOTO_POINT;
	
	  // 2) 명령줄 인자에 의한 초기화의 경우 
	  // goto_point_str = argv[i]; 


	// 1), 2) 공통  
	{
		// 도구에서 자동 생성 요 (시작)
			int numCnt = 0;
			vector<string>  strVector;

			string token;
			size_t delimiter_pos = 0;
			delimiter_pos = goto_point_str.find(",");

			while (delimiter_pos != string::npos)
			{
				numCnt++;
				token = goto_point_str.substr(0, delimiter_pos);
				goto_point_str = goto_point_str.substr (delimiter_pos+1);
				cout << "token: " << token << endl;
				strVector.push_back(token);
				cout << "goto_point_str: " << goto_point_str << endl;
				delimiter_pos = goto_point_str.find(",");
			}

			token = goto_point_str.substr(0, delimiter_pos);
			strVector.push_back(token);
			numCnt++;
			cout << "token: " << token << endl;
			cout << "numCnt: " << numCnt << endl;

		// 도구에서 자동 생성 요 (종료)

			string x_lat_str = strVector[0];
			goto_point.x_lat = atof (x_lat_str.c_str());

			string y_long_str = strVector[1];
			goto_point.y_long = atof (y_long_str.c_str());

			string z_alt_str = strVector[2];
			goto_point.z_alt = atof (z_alt_str.c_str());

			goto_cmd.request.goto_point = goto_point;
	} 
	

	 // goto_point parameter에 데이터 저장 완료 

	if (goto_client.call(goto_cmd) == true)
	{
		ROS_INFO("Goto command was sent to FC\n");
	}

	 // goto_point parameter에 데이터 저장 완료

	// 초기화 루틴 종료
	if (goto_cmd.response.value == true)
	{
		while (cur_phase.phase.compare ("READY")==0)
		{
			ros::spinOnce();
			rate.sleep();
			cout << "cur_phase: " << cur_phase_ptr->phase << endl;
		}
	}

	
---

