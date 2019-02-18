PRJ:projectName
SURVEYAREA_REF_SYSTEM:SURVEYAREA_REF_SYSTEM
SURVEYAREA_ALTITUDE:SURVEYAREA_ALTITUDE
SURVEYAREA_INTERVAL:SURVEYAREA_INTERVAL
SURVEYAREA_PTS:SURVEYAREA_PTS

---section_include
#include <eDrone_msgs/SurveyArea.h>
---

---section_main_service_msg_variables
	eDrone_msgs::SurveyArea surveyArea_cmd;
---

---section_main_service_client
	ros::ServiceClient surveyArea_client =nh.serviceClient<eDrone_msgs::SurveyArea>("srv_surveyArea");
---

---section_main_service_call	
	// SurveyArea	
	{
		while (cur_phase.phase.compare ("READY") !=0)
		{
			ros::spinOnce();
			rate.sleep();
		}

		ROS_INFO("We call SurveyArea API!"); // (수정)		
		
		surveyArea_cmd.request.surveyArea_ref_system = SURVEYAREA_REF_SYSTEM;
		surveyArea_cmd.request.surveyArea_altitude =  SURVEYAREA_ALTITUDE;
		surveyArea_cmd.request.surveyArea_interval = SURVEYAREA_INTERVAL;		
		std::string surveyArea_pts = SURVEYAREA_PTS;		
		{
			// 경로 정보가 포함된 문자열을 (상수 또는 매개변수)를 구분자 기준으로 나눈 후 string 벡터에 저장 (api 종류에 관계 없이 공통적으로 적용)
			int numCnt = 0;			
			vector<string>  strVector;

			string token;
			size_t delimiter_pos = 0;
			delimiter_pos = surveyArea_pts.find(",");
			
			while (delimiter_pos != string::npos)
			{
				numCnt++;
				token = surveyArea_pts.substr(0, delimiter_pos);
				surveyArea_pts = surveyArea_pts.substr (delimiter_pos+1);
				cout << "token: " << token << endl;
				strVector.push_back(token);
				cout << "surveyArea_pts: " << surveyArea_pts << endl;
				delimiter_pos = surveyArea_pts.find(",");
			}	
		
			token = surveyArea_pts.substr(0, delimiter_pos);
			strVector.push_back(token);
			numCnt++;
			cout << "token: " << token << endl;
			cout << "numCnt: " << numCnt << endl;


			// 문자열 벡터에 저장된 token들을 숫자로 변환 - 벡터 타입에 맞게 저장 (사용자가 코드 템플릿에 명시)
			int numPoints = strVector.size()/3;
			vector<Target> surveyArea_pts;

			if (SURVEYAREA_REF_SYSTEM == "ENU" || SURVEYAREA_REF_SYSTEM == "WGS84")
			{
				for (int i = 0; i <numPoints; i++)
				{
					eDrone_msgs::Target point;

					if (SURVEYAREA_REF_SYSTEM == "ENU")
					{
						point.ref_system = "ENU";
					}
					else if (SURVEYAREA_REF_SYSTEM == "WGS84")
					{
						point.ref_system = "WGS84";
					}

					string x_str = strVector[3*i];
					point.x_lat = atof (x_str.c_str());
					string y_str = strVector[3*i+1];
					point.y_long = atof (y_str.c_str());
					string z_str = strVector[3*i+2];
					point.z_alt = atof (z_str.c_str());
					surveyArea_pts.push_back(point);					
					cout << "point#" << i << "(" << point.x_lat <<", " << point.y_long << ", " << point.z_alt << ")" << endl;
				}

				surveyArea_cmd.request.surveyArea_pts = surveyArea_pts;
				//
			}					
			
		
	sleep(10);

		} // vector 파라미터 초기화 (surveyArea_pts)

		
		if (surveyArea_client.call(surveyArea_cmd) == true) // surveyArea 호출
		{
			ROS_INFO("SurveyArea command was sent to FC\n");
		}

		while (cur_phase.phase.compare ("READY") ==0)
		{
			ros::spinOnce();
			rate.sleep();
			cout << "cur_phase: " << cur_phase_ptr->phase << endl;
		}

	}
---
