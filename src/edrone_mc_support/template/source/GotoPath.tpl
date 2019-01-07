PRJ:projectName
GOTOPATH_REF_SYSTEM:GOTOPATH_REF_SYSTEM
GOTOPATH_ALTITUDE:GOTOPATH_ALTITUDE
GOTOPATH_PTS:GOTOPATH_PTS


---section_include
#include <eDrone_msgs/GotoPath.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_topic_callback_functions
---


---section_main_service_msg_variables
eDrone_msgs::GotoPath gotoPath_cmd;
---

---section_main_service_client
ros::ServiceClient gotoPath_client = nh.serviceClient<eDrone_msgs::GotoPath>("srv_gotoPath");
---

---section_main_service_call	
	// GotoPath

	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}

	string gotoPath_ref_system = ?[GOTOPATH_REF_SYSTEM];		
	gotoPath_cmd.request.gotoPath_ref_system = gotoPath_ref_system;

	double gotoPath_altitude = ?[GOTOPATH_ALTITUDE];	
	gotoPath_cmd.request.gotoPath_altitude = gotoPath_altitude;	

	vector<Target> gotoPath_pts; 
	string gotoPath_pts_str = ?[GOTOPATH_PTS];
	
	// 1) 상수를 이용한 초기화
	// (params.h에 다음 문장 생성) string GOTOPATH_PTS = <초기값>
	// (소스 파일) gotoPath_pts_str = GOTOPATH_PTS;

	// 2) 명령줄 인자를 이용한 초기화 
	// (소스 파일) gotoPath_pts_str = argv[i];
	
	// 1,2) 공통

	{
		// 도구에서 자동 생성 요 (시작)
			int numCnt = 0;
			vector<string>  strVector;

			string token;
			size_t delimiter_pos = 0;
			delimiter_pos = gotoPath_pts_str.find(",");

			while (delimiter_pos != string::npos)
			{
				numCnt++;
				token = gotoPath_pts_str.substr(0, delimiter_pos);
				gotoPath_pts_str = gotoPath_pts_str.substr (delimiter_pos+1);
				cout << "token: " << token << endl;
				strVector.push_back(token);
				cout << "gotoPath_pts_str: " << gotoPath_pts_str << endl;
				delimiter_pos = gotoPath_pts_str.find(",");
			}

			token = gotoPath_pts_str.substr(0, delimiter_pos);
			strVector.push_back(token);
			numCnt++;
			cout << "token: " << token << endl;
			cout << "numCnt: " << numCnt << endl;

		// 도구에서 자동 생성 요 (종료)

			int numPoints = strVector.size()/3;

			for (int i = 0; i <numPoints; i++)
			{
				Target point;
				string x_str = strVector[3*i];
				point.x_lat = atof (x_str.c_str());
				string y_str = strVector[3*i+1];
				point.y_long = atof (y_str.c_str());
				string z_str = strVector[3*i+2];
				point.z_alt = atof (z_str.c_str());
				gotoPath_pts.push_back(point);
				cout << "point#" << i << "(" << point.x_lat <<", " << point.y_long << ", " << point.z_alt << ")" << endl;
			}
				gotoPath_cmd.request.gotoPath_pts = gotoPath_pts;
	}  // gotoPath_pts 파라미터 초기화

	// 1,2) 공통

	gotoPath_client.call(gotoPath_cmd); // GotoPath API 호출

	if (gotoPath_cmd.response.value == true)
	{
		ROS_INFO("GotoPath API call - success!");
	}

	while (cur_phase.phase.compare ("READY")==0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}
	
	sleep(10);
---
