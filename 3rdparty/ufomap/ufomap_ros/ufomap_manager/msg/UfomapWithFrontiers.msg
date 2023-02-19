Header header

ufomap_msgs/UFOMap  ufomap

#geometry_msgs/Point[] local_frontiers_center
#geometry_msgs/Point[] global_frontiers_center
#
#int32 frontiers_depth

ufomap_manager/CellCode[] local_frontiers
ufomap_manager/CellCode[] global_frontiers
ufomap_manager/CellCode[] changed_cell_codes
ufomap_manager/CellCode[] known_cell_codes

# 构建ufomap时机器人的参数配置
string frame_id
string robot_base_frame_id
float64 robot_height
float64 robot_bottom
float64 robot_radius
float64 max_range

uint32 known_plane_cell_num