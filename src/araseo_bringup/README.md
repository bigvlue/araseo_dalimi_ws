# ARASEO-DALIMI 자율주행 로봇

미니어처 한국 도로 트랙에서 카메라 기반 자율주행을 수행하는 로봇 시스템입니다.

## 시스템 구성

| 항목 | 사양 |
|------|------|
| 보드 | Raspberry Pi 5 (Pinky Pro) |
| 모터 | Dynamixel XL330-M288-T x2 (/dev/ttyAMA4, 1Mbps) |
| 카메라 | Picamera2 OV5647 (640x480, 30fps) |
| LiDAR | SLAMTEC C1 (/dev/ttyAMA0, 460800baud) |
| 초음파 | I2C bus 1, addr 0x08 |
| IR 센서 | I2C bus 1, addr 0x08 |
| ROS | ROS 2 Jazzy |
| Robot ID | 14 |
| ROS_DOMAIN_ID | 204 |

## 빠른 시작

### 자율주행 테스트 모드

```bash
cd ~/araseo_dalimi_ws/src/araseo_bringup/scripts

# 시작
./start_autodrive.sh

# 브라우저에서 접속
# http://192.168.104.14:8089

# 정지
./stop_autodrive.sh
```

### 웹 인터페이스 사용법

| 버튼 | 동작 |
|------|------|
| **START / PAUSE** | 주행 시작 / 일시정지 |
| **LEFT** | 다음 교차로에서 좌회전 (교차로 감지 시 실행) |
| **RIGHT** | 다음 교차로에서 우회전 |
| **STRAIGHT** | 교차로에서 직진 (커브 보정 무시) |
| **AUTO** | 명령 취소, 자동 커브 따라가기 |
| **EMERGENCY STOP** | 즉시 정지 + 프로세스 종료 |

키보드 단축키: `A`=좌회전, `D`=우회전, `S`=시작/정지, `C`/`Space`=AUTO

### 화면 표시 정보

```
DRIVE CURVE-R(0.5) spd=45 turn=-12 w=420 road=72%
NEXT: NONE  L:45% R:32%
```

| 항목 | 설명 |
|------|------|
| DRIVE/STOP/PAUSED | 주행 상태 |
| CURVE-L/R(0.0~1.0) | 커브 방향 및 세기 |
| TURN-L/R | 교차로 회전 실행 중 |
| [WAIT:LEFT] | 회전 명령 대기 중 |
| spd | 모터 속도 |
| turn | 조향값 |
| w | 도로 폭 (px) |
| road | ROI 내 도로 비율 (%) |
| L/R % | 전방 좌/우 도로 비율 |
| \<L / R\> | 해당 방향 회전 가능 표시 (초록) |
| INTERSECTION | 교차로 감지 |

### ROS 2 전체 시스템 실행

```bash
cd ~/araseo_dalimi_ws/src/araseo_bringup/scripts

# tmux 세션으로 전체 시스템 시작 (robot_id=14)
./start_robot.sh 14

# 정지
./stop_robot.sh 14
```

## 자율주행 원리

### 도로 검출 (검은 도로 추종)

HSV 색공간에서 **검은 도로면**을 검출하고 나머지를 경계로 처리합니다.

```
도로 = S < 60 AND V < 150  (어둡고 채도 낮은 영역)
경계 = NOT 도로             (노란선, 흰선, 녹색, 벽 전부)
```

노란 중앙선은 별도 감지하여 노란색으로 표시합니다.

```
노란선 = H: 10~50, S > 20, V > 80
```

### 조향 제어

1. **조향 ROI** (y=350~450)에서 가장 넓은 연속 검은 구간의 중앙을 계산
2. **전방 주시점** (y=300)에서 동일하게 계산 (15% 가중)
3. 도로 중앙과 이미지 중앙의 차이로 조향값 산출
4. **커브 보정**: 커브 세기에 비례하여 바깥쪽으로 편향 (0.15~0.40)
5. **급커브 감속**: curve_strength > 0.5 시 최대 50% 감속

### 교차로 처리

- 전방 좌/우 영역의 검은색(도로) 비율로 회전 가능 여부 판단
- 30% 이상이면 해당 방향 회전 가능
- 회전 명령은 교차로 감지 시에만 실행, 통과 후 자동 소비

## 파라미터 조정

### 조명 변화 시 (필수)

`auto_drive.py` 상단의 값을 조정합니다:

```python
ROAD_S_MAX = 60    # 도로 최대 채도
ROAD_V_MAX = 150   # 도로 최대 밝기 ← 조명에 따라 조정
```

| 조명 | ROAD_V_MAX 권장 |
|------|----------------|
| 어두움 (V 평균 ~50) | 70~90 |
| 보통 (V 평균 ~80) | 100~120 |
| 밝음 (V 평균 ~105) | 130~150 |

### 조정 방법

```python
# 카메라 캡처 후 도로 영역 HSV 측정
python3 -c "
import sys, cv2, numpy as np, time
sys.path.insert(0, '/home/pinky/pinkylib/sensor/pinkylib')
from camera import Camera
cam = Camera(); cam.start(640, 480); time.sleep(2)
for _ in range(15): cam.get_frame()
frame = cam.get_frame()
r, g, b = cv2.split(frame)
g = np.clip(g.astype(np.float32) * 0.89, 0, 255).astype(np.uint8)
frame = cv2.merge([r, g, b])
cam.close()
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
road = hsv[350:450, 200:440, :]
print('도로 V: mean=%d min=%d max=%d' % (road[:,:,2].mean(), road[:,:,2].min(), road[:,:,2].max()))
print('도로 S: mean=%d min=%d max=%d' % (road[:,:,1].mean(), road[:,:,1].min(), road[:,:,1].max()))
"
```

ROAD_V_MAX는 도로 V max 값보다 약간 높게 설정합니다.

### 속도/조향 조정

```python
BASE_SPEED = 45    # 기본 속도 (Dynamixel raw, 30~60 권장)
MAX_TURN = 45      # 최대 조향 (30~60)
```

### 노란선 감지 조정

```python
YELLOW_LOW = np.array([10, 20, 80])    # H_min, S_min, V_min
YELLOW_HIGH = np.array([50, 255, 255]) # H_max, S_max, V_max
```

## 파일 구조

```
src/araseo_bringup/
├── scripts/
│   ├── auto_drive.py          # 자율주행 + 웹 모니터/제어
│   ├── start_autodrive.sh     # 자율주행 시작
│   ├── stop_autodrive.sh      # 자율주행 정지
│   ├── start_robot.sh         # ROS 2 전체 시스템 시작 (tmux)
│   ├── stop_robot.sh          # ROS 2 전체 시스템 정지
│   ├── install_service.sh     # systemd 서비스 설치
│   ├── dynamixel_driver_node.py
│   ├── ultrasonic_node.py
│   ├── camera_stream.py       # 카메라 스트림 (디버그용)
│   └── camera_teleop.py       # 텔레옵 (디버그용)
├── launch/
│   ├── robot.launch.py        # ROS 2 메인 런치
│   ├── fleet.launch.py        # 다중 로봇 런치
│   └── bev_calib.launch.py    # BEV 캘리브레이션
├── systemd/
│   └── araseo-robot.service   # systemd 자동 시작
├── config/
└── CMakeLists.txt
```

## 트러블슈팅

### 도로가 전부 마젠타로 표시됨
→ 조명 변화. `ROAD_V_MAX` 값을 올리세요.

### 노란선이 마젠타로 표시됨
→ 노란선 HSV 범위 조정. `YELLOW_LOW/HIGH` 값을 확인하세요.

### 모터가 정지 안 됨
→ `stop_autodrive.sh` 실행 또는 수동 정지:
```python
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyAMA4')
port.openPort(); port.setBaudRate(1000000)
ph = PacketHandler(2.0)
for mid in [1, 2]: ph.write4ByteTxRx(port, mid, 104, 0)
port.closePort()
"
```

### 카메라 점유 오류
→ 카메라 사용 중인 프로세스 종료:
```bash
pkill -9 -f "auto_drive\|camera_stream\|camera_teleop"
sleep 3  # libcamera 해제 대기
```

### GPS 수신 안 됨
→ `gps_field_msgs` 패키지가 빌드되어 있는지 확인:
```bash
source /opt/ros/jazzy/setup.bash
source ~/araseo_dalimi_ws/install/setup.bash
ros2 interface show gps_field_msgs/msg/PinkyGps
```
