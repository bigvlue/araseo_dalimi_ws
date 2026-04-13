# ARASEO-DALIMI 워크스페이스 — 프로그램 시작 가이드

로봇 IP: `192.168.4.1` (AP 모드) / `192.168.104.14` (외부 Wi-Fi)

---

## 1. 부팅 시 자동 실행 서비스

아래 서비스는 `systemd`로 등록되어 전원 ON 시 자동 시작됩니다.

| 서비스 | 설명 | 확인 명령 |
|--------|------|-----------|
| `pinky-ap.service` | AP 네트워크 설정 (`192.168.4.1`) | `systemctl status pinky-ap` |
| `pinky-boot.service` | 부팅 부저 알림 스크립트 | `systemctl status pinky-boot` |
| `jupyter.service` | Jupyter Notebook (포트 8888) | `systemctl status jupyter` |

Jupyter 접속: `http://192.168.4.1:8888`

---

## 2. 자율주행 (단독 실행 — 현재 가동 방식)

ROS2 없이 카메라 + Dynamixel 직접 제어로 검은 도로를 따라 주행합니다.

```bash
cd ~/araseo_dalimi_ws
python3 src/araseo_bringup/scripts/auto_drive.py
```

- 웹 모니터링: `http://192.168.4.1:8089`
- 긴급 정지: 웹 페이지의 **EMERGENCY STOP** 버튼 또는 `Ctrl+C`
- 주요 파라미터: `auto_drive.py` 상단 상수 (`BASE_SPEED`, `MAX_TURN`, `ROAD_V_MAX` 등)

---

## 3. 차선 검출 테스트

LAB + CLAHE 기반 차선 검출을 Flask 웹 UI로 확인합니다.

```bash
cd ~/araseo_dalimi_ws
python3 lane_detection.py
```

- 웹 접속: `http://192.168.4.1:5000`
- 실시간 파라미터 슬라이더로 튜닝 가능

---

## 4. ROS2 런치 (풀 스택)

ROS2 패키지 빌드 후 런치 파일로 전체 노드를 기동합니다.

### 빌드

```bash
cd ~/araseo_dalimi_ws
colcon build --symlink-install
source install/setup.bash
```

### 단일 로봇 실행

```bash
ros2 launch araseo_bringup robot.launch.py robot_id:=14
```

기동 순서 (자동):
1. 하드웨어 bringup (Dynamixel 모터 + IMU + LiDAR)
2. 자동 캘리브레이션
3. 캘리브레이션 완료 후 → GPS 브릿지, 휠 오도메트리, EKF, 차선 추종, 교통법, 미션 관리, HUD 순차 기동

### Fleet 실행 (최대 6대)

```bash
ros2 launch araseo_bringup fleet.launch.py robot_count:=6
```

### BEV 캘리브레이션

```bash
# 정지 이미지로 캘리브레이션 (권장)
ros2 launch araseo_bringup bev_calib.launch.py mode:=image image:=/path/to/frame.jpg

# 실시간 카메라로 캘리브레이션
ros2 launch araseo_bringup bev_calib.launch.py mode:=topic robot_id:=0

# 캘리브레이션 결과 검증
ros2 launch araseo_bringup bev_calib.launch.py mode:=verify image:=/path/to/frame.jpg
```

---

## 5. 정지 방법

### auto_drive.py 정지

- 웹 EMERGENCY STOP 버튼
- `Ctrl+C` (터미널)
- 프로세스 강제 종료 후 모터 직접 정지:
  ```bash
  kill $(pgrep -f auto_drive.py)
  # 모터가 멈추지 않으면 Dynamixel SDK로 직접 정지 필요
  python3 -c "
  from dynamixel_sdk import *
  port = PortHandler('/dev/ttyAMA4')
  port.openPort(); port.setBaudRate(1000000)
  ph = PacketHandler(2.0)
  for mid in [1, 2]:
      ph.write4ByteTxRx(port, mid, 104, 0)
      ph.write1ByteTxRx(port, mid, 64, 0)
  port.closePort()
  print('Motors stopped')
  "
  ```

### ROS2 노드 정지

```bash
# 전체 노드 종료
ros2 lifecycle set /dynamixel_driver shutdown
# 또는
Ctrl+C (런치 터미널)
```
