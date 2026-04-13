#!/bin/bash
# ARASEO-DALIMI 로봇 안전 종료
SESSION="araseo"
ROBOT_ID=${1:-14}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-204}

echo "로봇 정지 중 (robot_id=${ROBOT_ID})..."

# ROS2가 살아있으면 정지 명령 발행
if command -v ros2 &>/dev/null; then
    source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || true
    ros2 topic pub --once \
        /robot_${ROBOT_ID}/cmd_vel_safe \
        geometry_msgs/msg/Twist \
        '{linear: {x: 0.0}, angular: {z: 0.0}}' \
        2>/dev/null || true
fi

# tmux 세션 종료
tmux kill-session -t "${SESSION}" 2>/dev/null && \
    echo "tmux 세션 종료 완료" || \
    echo "실행 중인 세션 없음"
