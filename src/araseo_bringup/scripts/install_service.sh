#!/bin/bash
# systemd 서비스 설치 스크립트
# 사용법: sudo ./install_service.sh [robot_id] [ros_distro] [ws_path]

ROBOT_ID=${1:-14}
ROS_DISTRO=${2:-jazzy}
WS_PATH=${3:-${HOME}/araseo_dalimi_ws}
SERVICE_SRC="$(dirname $(realpath $0))/../systemd/araseo-robot.service"
SERVICE_DST="/etc/systemd/system/araseo-robot.service"
USER_NAME=$(logname 2>/dev/null || echo "pi")

if [ "$EUID" -ne 0 ]; then
    echo "[오류] sudo 로 실행하세요: sudo ./install_service.sh"
    exit 1
fi

echo "설치 설정:"
echo "  robot_id : ${ROBOT_ID}"
echo "  ROS      : ${ROS_DISTRO}"
echo "  WS 경로  : ${WS_PATH}"
echo "  사용자   : ${USER_NAME}"

# 서비스 파일 복사 + 변수 치환
sed -e "s|pi|${USER_NAME}|g" \
    -e "s|/home/pi|${HOME}|g" \
    -e "s|ROBOT_ID=14|ROBOT_ID=${ROBOT_ID}|g" \
    -e "s|ROS_DISTRO=jazzy|ROS_DISTRO=${ROS_DISTRO}|g" \
    -e "s|WS_PATH=/home/pi/araseo_dalimi_ws|WS_PATH=${WS_PATH}|g" \
    "${SERVICE_SRC}" > "${SERVICE_DST}"

systemctl daemon-reload
systemctl enable araseo-robot.service
systemctl start  araseo-robot.service

echo ""
echo "설치 완료."
echo "  상태 확인: systemctl status araseo-robot"
echo "  로그 확인: journalctl -u araseo-robot -f"
echo "  세션 접속: tmux attach -t araseo"
echo "  자동시작 해제: sudo systemctl disable araseo-robot"
