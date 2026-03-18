# 无人机状态消息
std_msgs/Header header

# 无人机ID: "carrier" 或 "mini"
string uav_id

# 位姿
geometry_msgs/Pose pose

# 速度
geometry_msgs/Twist twist

# 电池电量 (%)
float64 battery_level

# GPS状态
bool gps_valid

# 对接状态
bool is_docked
