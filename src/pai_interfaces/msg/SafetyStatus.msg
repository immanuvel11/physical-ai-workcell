std_msgs/Header header
string state                 # NORMAL | SLOW | STOP | ESTOP
float32 speed_scale          # 0..1 multiplier the skills must respect
float32 min_human_distance   # metres (inf if nobody detected)
string reason
