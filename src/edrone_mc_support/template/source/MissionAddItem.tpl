
PRJ:projectName
MISSIONADDITEM_WAYPOINT:MISSIONADDITEM_WAYPOINT

                        ---section_include,1
                        #include <eDrone_msgs/MissionAddItem.h>
                        ---

                        ---section_namespace,1
                        using namespace mavros_msgs;
---

---section_pointer_variables
---

---section_functions,1
void print_waypoints (vector<mavros_msgs::Waypoint> waypoints) // 웨이포인트 정보
{
    for (int i = 0; i < waypoints.size(); i++)
    {
        cout << "waypoint[" << i << "]: ";

        cout << endl ;

        cout << waypoints[i] << endl;
    }
}
---


---section_topic_callback_functions
---

---section_main_init
---

---section_main_topic_msg_variables

---

---section_main_service_msg_variables,1
eDrone_msgs::MissionAddItem missionAddItem_cmd;
---

---section_main_variables

---

---section_main_topic_subscriber
---

---section_main_service_client,1
ros::ServiceClient missionAddItem_client =nh.serviceClient<eDrone_msgs::MissionAddItem>("srv_missionAddItem");
---

---section_main_service_call
// MissionAddItem
{
    Waypoint missionAddItem_waypoint;
    string missionAddItem_waypoint_str = ?[MISSIONADDITEM_WAYPOINT];

    // Waypint가 구조체이므로 missionAddItem_waypoint을 초기화 하는데 필요한 문자열 변수 (missionAddItem_waypoint_str을)가 선언되었다는 가정 하에 아래 코드 생성

    // 1) 상수에 의한 초기화의 경우
    // (params.h에 선언) string MISSIONADDITEM_WAYPOINT = <초기값> (예: "3,16,10,10,10");
    // missionAddItem_waypoint_str = MISSIONADDITEM_WAYPOINT;

    // 2) 명령줄 인자에 의한 초기화의 경우
    // missionAddItem_waypoint_str = argv[i];

    // 도구에서 자동 생성 요 (시작)
    vector<string>  strVector;

    string token;
    size_t delimiter_pos = 0;
    delimiter_pos = missionAddItem_waypoint_str.find(",");

    while (delimiter_pos != string::npos)
    {

        token = missionAddItem_waypoint_str.substr(0, delimiter_pos);
        missionAddItem_waypoint_str = missionAddItem_waypoint_str.substr (delimiter_pos+1);
        cout << "token: " << token << endl;
        strVector.push_back(token);
        cout << "missionAddItem_waypoint_str: " << missionAddItem_waypoint_str << endl;
        delimiter_pos = missionAddItem_waypoint_str.find(",");
    }

    token = missionAddItem_waypoint_str.substr(0, delimiter_pos);
    strVector.push_back(token);

    cout << "token: " << token << endl;


    // 도구에서 자동 생성 요 (종료)

    string frame_str = strVector[0];
    int frame = atoi (frame_str.c_str());
    missionAddItem_waypoint.frame = frame;

    string command_str = strVector[1];
    int command = atoi (command_str.c_str());
    missionAddItem_waypoint.command = command;

    string x_lat_str = strVector[2];
    missionAddItem_waypoint.x_lat = atof (x_lat_str.c_str());

    string y_long_str = strVector[3];
    missionAddItem_waypoint.y_long = atof (y_long_str.c_str());

    string z_alt_str = strVector[4];
    missionAddItem_waypoint.z_alt = atof (z_alt_str.c_str());

    missionAddItem_waypoint.autocontinue = true;
    missionAddItem_cmd.request.missionAddItem_waypoint = missionAddItem_waypoint;
} // missionAddItem_waypoint parameter에 데이터 저장 완료

if ( missionAddItem_client.call(missionAddItem_cmd) == true)
{
    ROS_INFO("missionAddItem command was sent");
}

---

