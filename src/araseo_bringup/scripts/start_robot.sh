#!/bin/bash
# ─────────────────────────────────────────────────────────────
# ARASEO-DALIMI 로봇 tmux 자동 런처
# 사용법: ./start_robot.sh [robot_id]
#         robot_id 기본값: 14
# 단축키: Ctrl+b → 숫자키로 윈도우 전환
#         Ctrl+b → q  로 패널 번호 표시
#         Ctrl+b → d  로 세션 백그라운드 전환
# ─────────────────────────────────────────────────────────────
set -e

ROBOT_ID=${1:-14}
SESSION="araseo"
ROS_DISTRO=${ROS_DISTRO:-jazzy}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-204}

# 워크스페이스 경로 자동 탐색
if [ -f "${HOME}/araseo_dalimi_ws/install/setup.bash" ]; then
    WS_SETUP="${HOME}/araseo_dalimi_ws/install/setup.bash"
elif [ -f "$(dirname $(dirname $(realpath $0)))/../../install/setup.bash" ]; then
    WS_SETUP="$(dirname $(dirname $(realpath $0)))/../../install/setup.bash"
else
    echo "[오류] 워크스페이스 install/setup.bash 를 찾을 수 없습니다."
    exit 1
fi

# ROS2 source 헬퍼 문자열
SRC="source ${ROS_SETUP} && source ${WS_SETUP} && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

echo "────────────────────────────────────────"
echo "  ARASEO-DALIMI 로봇 시작"
echo "  robot_id : ${ROBOT_ID}"
echo "  ROS      : ${ROS_DISTRO}"
echo "  DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  WS       : ${WS_SETUP}"
echo "────────────────────────────────────────"

# 기존 세션 정리
tmux kill-session -t "${SESSION}" 2>/dev/null || true

# ── Window 0: launch ─────────────────────────────────────────
tmux new-session -d -s "${SESSION}" -n "launch" -x 220 -y 50

tmux send-keys -t "${SESSION}:launch" \
    "${SRC} && ros2 launch araseo_bringup robot.launch.py robot_id:=${ROBOT_ID}" \
    Enter

# ── Window 1: monitor (4분할) ─────────────────────────────────
tmux new-window -t "${SESSION}" -n "monitor"

# 좌상: cmd_vel_safe
tmux send-keys -t "${SESSION}:monitor" \
    "${SRC} && watch -n 0.3 \"ros2 topic echo /robot_${ROBOT_ID}/cmd_vel_safe --once 2>/dev/null || echo '토픽 없음'\"" \
    Enter

# 우상: lane_info + traffic_sign
tmux split-window -h -t "${SESSION}:monitor"
tmux send-keys -t "${SESSION}:monitor" \
    "${SRC} && ros2 topic echo /robot_${ROBOT_ID}/traffic_sign 2>/dev/null" \
    Enter

# 좌하: GPS 위치
tmux select-pane -t "${SESSION}:monitor.0"
tmux split-window -v -t "${SESSION}:monitor.0"
tmux send-keys -t "${SESSION}:monitor" \
    "${SRC} && watch -n 0.5 \"ros2 topic echo /robot_${ROBOT_ID}/ekf_pose --once 2>/dev/null | grep -E 'position|orientation' || echo '위치 없음'\"" \
    Enter

# 우하: topic hz 통계
tmux select-pane -t "${SESSION}:monitor.1"
tmux split-window -v -t "${SESSION}:monitor.1"
tmux send-keys -t "${SESSION}:monitor" \
    "watch -n 2 \"${SRC} > /dev/null 2>&1; echo '=== Topic Hz ===' && ros2 topic hz /robot_${ROBOT_ID}/cmd_vel_safe 2>/dev/null & sleep 1.8; kill %1 2>/dev/null; echo '--- Camera ---'; ros2 topic hz /robot_${ROBOT_ID}/camera/image_raw 2>/dev/null & sleep 1.8; kill %1 2>/dev/null\"" \
    Enter

# 레이아웃 정리
tmux select-layout -t "${SESSION}:monitor" tiled

# ── Window 2: system ─────────────────────────────────────────
tmux new-window -t "${SESSION}" -n "system"

# 상단: htop
tmux send-keys -t "${SESSION}:system" \
    "htop" \
    Enter

# 하단: ROS2 노드 목록
tmux split-window -v -t "${SESSION}:system" -p 30
tmux send-keys -t "${SESSION}:system" \
    "${SRC} && watch -n 2 'ros2 node list 2>/dev/null'" \
    Enter

# ── Window 3: shell ───────────────────────────────────────────
tmux new-window -t "${SESSION}" -n "shell"
tmux send-keys -t "${SESSION}:shell" \
    "${SRC} && echo '✓ ROS2 환경 준비됨 (robot_id=${ROBOT_ID})'" \
    Enter

# launch 창으로 포커스 이동 후 attach
tmux select-window -t "${SESSION}:launch"
echo ""
echo "세션 시작 완료. 접속 중..."
echo "(Ctrl+b → d 로 백그라운드 전환, tmux attach -t ${SESSION} 으로 재접속)"
echo ""
tmux attach-session -t "${SESSION}"
