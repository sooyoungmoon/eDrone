PRJ:projectName
NOFLYZONESET_REF_SYSTEM:NOFLYZONESET_REF_SYSTEM
NOFLYZONESET_PTS:NOFLYZONESET_PTS

---section_include
#include <eDrone_msgs/NoflyZoneSet.h>
---

---section_namespace
---

---section_pointer_variables
---

---section_topic_callback_functions
---


---section_main_service_msg_variables
	eDrone_msgs::NoflyZoneSet noflyZoneSet_cmd;
---

---section_main_service_client
	ros::ServiceClient noflyZoneSet_Client<eDrone_msgs::NoflyZoneSet>("srv_noflyZoneSet");
---

---section_main_service_call	

	
	string nfset_ref_system = ?[NOFLYZONESET_REF_SYSTEM];
	noflyZoneSet_cmd.request.noflyZoneSet_ref_system = nfset_ref_system;
	
	vector<Target> noflyZoneSet_pts; 
	string noflyZoneSet_pts_str = ?[NOFLYZONESET_PTS];


	// 1) 상수를 이용한 초기화
	// (params.h에 다음 문장 생성) string GOTOPATH_PTS_STR = <초기값>
	// (소스 파일) noflyZoneSet_pts_str = GOTOPATH_PTS_STR;

	// 2) 명령줄 인자를 이용한 초기화 
	// (소스 파일) noflyZoneSet_pts_str = argv[i];	

	// 공통 
	{ 
		// 도구에서 자동 생성 요 (시작)
			int numCnt = 0;
			vector<string>  strVector;

			string token;
			size_t delimiter_pos = 0;
			delimiter_pos = noflyZoneSet_pts_str.find(",");

			while (delimiter_pos != string::npos)
			{
				numCnt++;
				token = noflyZoneSet_pts_str.substr(0, delimiter_pos);
				noflyZoneSet_pts_str = noflyZoneSet_pts_str.substr (delimiter_pos+1);
				cout << "token: " << token << endl;
				strVector.push_back(token);
				cout << "noflyZoneSet_pts_str: " << noflyZoneSet_pts_str << endl;
				delimiter_pos = noflyZoneSet_pts_str.find(",");
			}

			token = noflyZoneSet_pts_str.substr(0, delimiter_pos);
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
				noflyZoneSet_pts.push_back(point);
				cout << "point#" << i << "(" << point.x_lat <<", " << point.y_long << ", " << point.z_alt << ")" << endl;
			}
				noflyZoneSet_cmd.request.noflyZoneSet_pts = noflyZoneSet_pts;
	}  // gotoPath_pts 파라미터 초기화
---
