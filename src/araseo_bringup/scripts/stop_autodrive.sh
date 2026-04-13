#!/bin/bash
# ─────────────────────────────────────────────────────────────
# ARASEO-DALIMI 자율주행 테스트 정지
# ─────────────────────────────────────────────────────────────

echo "자율주행 정지 중..."

# 모터 안전 정지 (최우선)
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyAMA4')
port.openPort(); port.setBaudRate(1000000)
ph = PacketHandler(2.0)
for mid in [1, 2]: ph.write4ByteTxRx(port, mid, 104, 0)
port.closePort(); print('  모터 정지 완료')
" 2>/dev/null || echo "  모터 정지 실패 (포트 점유)"

# 프로세스 종료
pkill -9 -f auto_drive.py 2>/dev/null && echo "  프로세스 종료 완료" || echo "  실행 중인 프로세스 없음"

# 포트 해제
fuser -k 8089/tcp 2>/dev/null || true

echo "정지 완료."
