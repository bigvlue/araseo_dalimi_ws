#!/bin/bash
# ─────────────────────────────────────────────────────────────
# ARASEO-DALIMI 자율주행 테스트 시작
# 사용법: ./start_autodrive.sh
# 웹 모니터: http://<robot_ip>:8089
# ─────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="/tmp"

echo "────────────────────────────────────────"
echo "  ARASEO-DALIMI 자율주행 테스트"
echo "────────────────────────────────────────"

# 기존 프로세스 정리
echo "[1/3] 기존 프로세스 정리..."
pkill -9 -f auto_drive.py 2>/dev/null || true
fuser -k 8089/tcp 2>/dev/null || true

# 모터 안전 정지
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyAMA4')
port.openPort(); port.setBaudRate(1000000)
ph = PacketHandler(2.0)
for mid in [1, 2]: ph.write4ByteTxRx(port, mid, 104, 0)
port.closePort()
" 2>/dev/null || true

sleep 2

# 자율주행 시작
echo "[2/3] 자율주행 서버 시작..."
PYTHONUNBUFFERED=1 python3 "${SCRIPT_DIR}/auto_drive.py" > "${LOG_DIR}/auto_drive.log" 2>&1 &
PID=$!
echo "  PID: ${PID}"

sleep 5

# 확인
if kill -0 $PID 2>/dev/null; then
    echo "[3/3] 시작 완료!"
    echo ""
    echo "  웹 모니터: http://$(hostname -I | awk '{print $1}'):8089"
    echo "  로그 확인: tail -f ${LOG_DIR}/auto_drive.log"
    echo "  정지:      ./stop_autodrive.sh"
    echo ""
    echo "  웹에서 START 버튼을 눌러 주행 시작"
else
    echo "[오류] 시작 실패. 로그 확인:"
    tail -10 "${LOG_DIR}/auto_drive.log"
    exit 1
fi
