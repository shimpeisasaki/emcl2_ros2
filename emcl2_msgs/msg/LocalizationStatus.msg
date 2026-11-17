# header.stamp is publication time; scan_stamp is the actual evaluated observation.
std_msgs/Header header
builtin_interfaces/Time scan_stamp
uint8 INITIALIZING=0
uint8 TRACKING=1
uint8 LOST=2
uint8 RECOVERING=3
uint8 UNAVAILABLE=4
uint8 state
string instance_id
uint64 transition_id
float64 alpha
string reason
