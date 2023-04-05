#!/bin/bash
rosbag record --split --size=4096 -b 1024 -o $1 \
/tf_static \
/tf \
/rosout \
/camera/color/image_raw_throttle/compressed \
/os_node/lidar_packets \
/os_node/imu_packets \
/os_node/metadata \
/imu/data_raw  \
/navsat/fix \
/jackal_velocity_controller/cmd_vel \
/jackal_teleop/is_auto \
/object_mapper/map \
/top_down_render/pose_est \
/spomp_local/reachability \
/spomp_local/control_viz \
/spomp_global/graph_viz \
/spomp_global/path_viz \
/spomp_global/reachability_history \
/spomp_global/aerial_map_trav_viz/compressed \
/top_down_render/map_viz/compressed \
/os_node/rofl_odom/pose \
/goal_manager/goal_viz \
/goal_manager/claimed_goals \
/find_object_goal_gen/target_goals \
/europa/top_down_render/pose_est \
/europa/object_mapper/map \
/europa/spomp_global/reachability_history \
/europa/spomp_global/path_viz \
/europa/goal_manager/claimed_goals \
/io/top_down_render/pose_est \
/io/object_mapper/map \
/io/spomp_global/reachability_history \
/io/spomp_global/path_viz \
/io/goal_manager/claimed_goals \
/callisto/top_down_render/pose_est \
/callisto/object_mapper/map \
/callisto/spomp_global/reachability_history \
/callisto/spomp_global/path_viz \
/callisto/goal_manager/claimed_goals \
/titan/asoom/map/compressed \
/titan/asoom/recent_key_pose \
/ddb/rajant/log \
/ddb/rajant/rssi/titan \
/ddb/rajant/rssi/basestation \
/ddb/rajant/rssi/callisto \
/ddb/rajant/rssi/europa \
/ddb/rajant/rssi/io \
/ddb/topic_publisher/headers \
/ddb/client_sync_complete/titan \
/ddb/client_sync_complete/basestation \
/ddb/client_sync_complete/callisto \
/ddb/client_sync_complete/europa \
/ddb/client_sync_complete/io \
/ddb/server_sync_complete/titan \
/ddb/server_sync_complete/basestation \
/ddb/server_sync_complete/callisto \
/ddb/server_sync_complete/europa \
/ddb/server_sync_complete/io \
/ddb/client_sm_state/callisto \
/ddb/client_sm_state/europa  \
/ddb/client_sm_state/io \
/ddb/client_sm_state/titan \
/ddb/client_sm_state/basestation \
/ddb/client_stats/callisto \
/ddb/client_stats/europa  \
/ddb/client_stats/io \
/ddb/client_stats/titan \
/ddb/client_stats/basestation
