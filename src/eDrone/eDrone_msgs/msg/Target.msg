# The target information (coordinates)
# target_seq_no - the sequence number of targets
# is_global - true (WGS84) or false (ENU)
# ref_system - "WGS84" OR "ENU")
# x_lat - x (ENU) or latitude (WGS84)
# y_long - y (ENU) or longitude (WGS84)
# z_alt - z (ENU) or altitude (WGS84)
# reached - true (UAV has reached at the target) or false 

int16 target_seq_no 
bool is_global
string ref_system
float64 x_lat
float64 y_long
float64 z_alt
bool reached


