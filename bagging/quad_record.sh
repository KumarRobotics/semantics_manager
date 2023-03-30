#!/bin/sh
rosbag record -o $1 \
/ovc/imu \
/ovc/rgb/camera_info \
/ovc/rgb/image_color/compressed \
/ovc/vectornav/imu \
/ovc/vectornav/mag \
/ovc_embedded_driver_node/parameter_descriptions \
/ovc_embedded_driver_node/parameter_updates \
/mavlink/from \
/mavlink/gcs_ip \
/mavlink/to \
/mavros/actuator_control \
/mavros/adsb/send \
/mavros/adsb/vehicle \
/mavros/altitude \
/mavros/battery \
/mavros/battery2 \
/mavros/cam_imu_sync/cam_imu_stamp \
/mavros/companion_process/status \
/mavros/debug_value/debug \
/mavros/debug_value/debug_vector \
/mavros/debug_value/named_value_float \
/mavros/debug_value/named_value_int \
/mavros/debug_value/send \
/mavros/esc_info \
/mavros/esc_status \
/mavros/esc_telemetry \
/mavros/estimator_status \
/mavros/extended_state \
/mavros/fake_gps/mocap/tf \
/mavros/geofence/waypoints \
/mavros/global_position/compass_hdg \
/mavros/global_position/global \
/mavros/global_position/gp_lp_offset \
/mavros/global_position/gp_origin \
/mavros/global_position/home \
/mavros/global_position/local \
/mavros/global_position/raw/fix \
/mavros/global_position/raw/gps_vel \
/mavros/global_position/raw/satellites \
/mavros/global_position/rel_alt \
/mavros/global_position/set_gp_origin \
/mavros/gps_input/gps_input \
/mavros/gps_rtk/rtk_baseline \
/mavros/gps_rtk/send_rtcm \
/mavros/gpsstatus/gps1/raw \
/mavros/gpsstatus/gps1/rtk \
/mavros/gpsstatus/gps2/raw \
/mavros/gpsstatus/gps2/rtk \
/mavros/hil/actuator_controls \
/mavros/hil/controls \
/mavros/hil/gps \
/mavros/hil/imu_ned \
/mavros/hil/optical_flow \
/mavros/hil/rc_inputs \
/mavros/hil/state \
/mavros/home_position/home \
/mavros/home_position/set \
/mavros/imu/data \
/mavros/imu/data_raw \
/mavros/imu/diff_pressure \
/mavros/imu/mag \
/mavros/imu/static_pressure \
/mavros/imu/temperature_baro \
/mavros/imu/temperature_imu \
/mavros/landing_target/lt_marker \
/mavros/landing_target/pose \
/mavros/landing_target/pose_in \
/mavros/local_position/accel \
/mavros/local_position/odom \
/mavros/local_position/pose \
/mavros/local_position/pose_cov \
/mavros/local_position/velocity_body \
/mavros/local_position/velocity_body_cov \
/mavros/local_position/velocity_local \
/mavros/log_transfer/raw/log_data \
/mavros/log_transfer/raw/log_entry \
/mavros/mag_calibration/report \
/mavros/mag_calibration/status \
/mavros/manual_control/control \
/mavros/manual_control/send \
/mavros/mission/reached \
/mavros/mission/waypoints \
/mavros/mocap/pose \
/mavros/mount_control/command \
/mavros/mount_control/orientation \
/mavros/mount_control/status \
/mavros/nav_controller_output \
/mavros/obstacle/send \
/mavros/odometry/in \
/mavros/odometry/out \
/mavros/onboard_computer/status \
/mavros/param/param_value \
/mavros/play_tune \
/mavros/px4flow/ground_distance \
/mavros/px4flow/raw/optical_flow_rad \
/mavros/px4flow/raw/send \
/mavros/px4flow/temperature \
/mavros/radio_status \
/mavros/rallypoint/waypoints \
/mavros/rc/in \
/mavros/rc/out \
/mavros/rc/override \
/mavros/setpoint_accel/accel \
/mavros/setpoint_attitude/cmd_vel \
/mavros/setpoint_attitude/thrust \
/mavros/setpoint_position/global \
/mavros/setpoint_position/global_to_local \
/mavros/setpoint_position/local \
/mavros/setpoint_raw/attitude \
/mavros/setpoint_raw/global \
/mavros/setpoint_raw/local \
/mavros/setpoint_raw/target_attitude \
/mavros/setpoint_raw/target_global \
/mavros/setpoint_raw/target_local \
/mavros/setpoint_trajectory/desired \
/mavros/setpoint_trajectory/local \
/mavros/setpoint_velocity/cmd_vel \
/mavros/setpoint_velocity/cmd_vel_unstamped \
/mavros/state \
/mavros/statustext/recv \
/mavros/statustext/send \
/mavros/target_actuator_control \
/mavros/time_reference \
/mavros/timesync_status \
/mavros/trajectory/desired \
/mavros/trajectory/generated \
/mavros/trajectory/path \
/mavros/vfr_hud \
/mavros/vision_pose/pose \
/mavros/vision_pose/pose_cov \
/mavros/vision_speed/speed_twist_cov \
/mavros/wind_estimation \
/rosout \
/rosout_agg \
/rtcm \
/ublox/fix \
/ublox/fix_velocity \
/asoom/viz \
/asoom/recent_cloud \
/asoom/recent_key_pose \
/asoom/map/compressed \
/orbslam3_ros_node/pose \
/clock \
/air_router/goal \
/air_router/navigator/state \
/air_router/navigator/viz/compressed \
/air_router/start \
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
/basestation/target_goals \
/ddb/rajant/log \
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
/ddb/sm_state/callisto \
/ddb/sm_state/europa  \
/ddb/sm_state/io \
/ddb/sm_state/titan \
/ddb/sm_state/basestation
