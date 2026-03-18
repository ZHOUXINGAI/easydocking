# 对接状态消息
std_msgs/Header header

# 当前阶段: IDLE, APPROACH, TRACKING, DOCKING, COMPLETED, FAILED
string phase

# 是否活跃
bool is_active

# 相对距离
float64 relative_distance

# 相对位置
geometry_msgs/Vector3 relative_position

# 相对速度
geometry_msgs/Vector3 relative_velocity

# 干扰估计
geometry_msgs/Vector3 disturbance_estimate

# 估计的目标速度
geometry_msgs/Vector3 estimated_target_velocity
