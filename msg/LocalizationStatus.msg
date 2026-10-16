# SDK 补充定义：PDF 仅规定 /nav/status/localization 为 10 Hz Topic。
std_msgs/Header header              # SDK 状态发布时间，不表示后端数据新鲜度
bool backend_connected             # 状态与地图回显话题存在发布者，不代表数据新鲜
bool map_loaded                    # 本 SDK 发图后收到后端地图回显及 waiting_initialpose
string map_path
string map_md5
bool tracking                      # 是否有本 SDK 的活动重定位 Action
string state
bool match_success                 # 后端 localized 且本轮有效变换已收到
float64 fitness_score
float64 threshold
uint32 attempt_count
float64 elapsed_time
string message
bool transform_valid
geometry_msgs/TransformStamped match_transform  # 保留后端变换的采样时间
