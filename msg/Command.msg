# Command.msg
# 用于发送给 Manager 节点的命令

Header header             # 标准的 ROS 消息头部
string command_type       # 指令类型，可选值：
                          #   "start_backend" - 按依赖顺序启动当前配置中的后端模块
                          #   "stop_backend"  - 逆序停止当前配置中由 Manager 管理的后端模块
string[] target_launches  # 保留字段，当前 start_backend/stop_backend 忽略
float64[] parameters      # 可选命令参数
string extra_data         # 额外指令数据
