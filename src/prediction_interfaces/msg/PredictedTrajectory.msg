# 单个目标的预测轨迹（车体/lidar_link 坐标系，绝对坐标）
int32 id
string type
float32 dt          # 预测步长（秒）
float32[] future_x  # 未来 N 步 x（绝对坐标）
float32[] future_y  # 未来 N 步 y（绝对坐标）
