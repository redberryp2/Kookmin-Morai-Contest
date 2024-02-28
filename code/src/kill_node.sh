#!/bin/bash

# 노드 이름 목록
node_names=(
    
    

    "/ackermann_to_vesc"
    "/throttle_interpolator"
    
)

# 노드가 실행 중인지 확인하고 강제 종료
for node_name in "${node_names[@]}"; do
    if rosnode list | grep -q "$node_name"; then
        echo "Killing node: $node_name"
        rosnode kill "$node_name"
    else
        echo "Node not found: $node_name"
    fi
done


# "/amcl" "/mux_chainer"
#     "/base_list_to_imu"
#     "/ackermann_to_lidar"
#     "/cmd_vel_to_ackermann"
#     "/ekf_se_odom"
#     "/high_level/ackermann_cmd_mux"
#     "/high_level/ackermann_cmd_mux_nodelet_manager"
#     "/lidar_convert"
#     "/low_level/ackermann_cmd_mux"
#     "/low_level/ackermann_cmd_mux_nodelet_manager"
#     "/map_server"
#     "/move_base"
#     "/mux_chainer"
#     "/mux_topic_backward_compat_navigation"
#     "/mux_topic_backward_compat_safety"
#     "/mux_topic_backward_compat_teleop"
#     "/navigation_client"
#     "/rviz"
#     "/vesc_to_odom"

# "/move_base"
    # "/cmd_vel_to_ackermann"
    # "/ekf_se_odom"
    # "/mux_topic_backward_compat_navigation"
    # "/mux_topic_backward_compat_safety"
    # "/mux_topic_backward_compat_teleop"
    # "/navigation_client"
    # "/base_list_to_imu"