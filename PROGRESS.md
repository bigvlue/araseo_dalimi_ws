# ARASEO-DALIMI 프로젝트 진행 기록

> 최종 업데이트: 2026-04-13
> 로봇 ID: 14 | ROS_DOMAIN_ID: 204 | 플랫폼: Pinky Pro (Raspberry Pi)

---

## 1. 프로젝트 개요

미니어처 도로 트랙(1880×1410mm) 위에서 자율주행하는 교육용 로봇 시스템.
최대 6대 Fleet 운용을 목표로 하며, ROS2 Jazzy 기반 풀 스택과 ROS2 없이 단독 실행 가능한 경량 모드를 모두 지원한다.

---

## 2. 완성된 ROS2 패키지 (9개)

### 2-1. araseo_dalimi_interfaces — 메시지/서비스/액션 정의
- `RobotPose.msg` — 위치 (x_mm, y_mm, theta_mrad, confidence, source)
- `PinkyGps.msg` — 호스트 PC GPS 카메라 출력
- `LaneInfo.msg` — 차선 검출 결과 (lateral/heading error, 정지선/교차로 플래그)
- `TrafficSign.msg` — 교통 표지 (STOP_LINE, INTERSECTION, SPEED_LIMIT, NO_ENTRY)
- `MissionStatus.msg` — 미션 상태 (IDLE/RUNNING/PAUSED/COMPLETED/FAILED)
- `FleetStatus.msg` — 다중 로봇 상태 배열
- `SetMission.srv` / `GetMissionStatus.srv` / `SetDrivingMode.srv`
- `NavigateToGoal.action` — Nav2 웨이포인트 내비게이션

### 2-2. araseo_bringup — 하드웨어 기동 및 런치
- `robot.launch.py` — 단일 로봇 전체 기동 (캘리브레이션 → 센서 → 제어 순차 실행)
- `fleet.launch.py` — 6대 Fleet 동시 기동
- `bev_calib.launch.py` — BEV+ROI 캘리브레이션 GUI 도구
- `dynamixel_driver_node.py` — XL330 모터 드라이버 (속도 제어 모드)
- `ultrasonic_node.py` — 초음파 거리 측정
- `camera_stream.py` / `camera_teleop.py` — 카메라 스트리밍 및 수동 조종
- `auto_drive.py` — ROS2 없이 단독 자율주행 + 웹 모니터링 (포트 8089)
- `start_robot.sh` / `stop_robot.sh` — tmux 세션 관리 스크립트
- `araseo-robot.service` — systemd 자동 시작 서비스

### 2-3. araseo_gps_camera — GPS 브릿지
- 호스트 PC `/pinky_{id}/gps_pos` → `/robot_{id}/pose` (RobotPose) 변환
- GPS 타임아웃 시 odom 폴백 (0.5초)
- 맵 범위 클램핑 (1880×1410mm)

### 2-4. araseo_lane_following — 차선 검출 및 추종
- BEV 원근 변환 (8포인트 호모그래피)
- HSV 색상 분할 (노란 중앙선, 흰 차선)
- 슬라이딩 윈도우 차선 검출 (9개 윈도우, 마진 60px)
- Pure Pursuit 조향 (전방주시 직진 0.30m, 커브 0.15m)
- 정지선 감지 (흰색 픽셀 임계값 80px)
- 교차로 감지 (노란색 밀도 기반)
- IR 센서 보조 (TCRT5000 × 3)
- `bev_calibrator.py` — 인터랙티브 캘리브레이션 도구
- 기본 속도: 0.15 m/s, 최대 조향: 30°

### 2-5. araseo_traffic_law — 교통법규 준수
- 7-상태 머신: NORMAL → APPROACHING_STOP → STOP_HOLD(2초) → INTERSECTION → YIELD → FOLLOWING → COLLISION_AVOID
- LiDAR 장애물 감속(400mm) / 정지(200mm), FOV 90°
- 초음파 정지 거리 150mm
- 앞차 추종 거리 400mm
- 중앙선 침범 경고 (0.65 이상)
- cmd_vel → cmd_vel_safe 변환 (속도 스케일링)

### 2-6. araseo_localization — 위치 추정
- `auto_calibration_node.py` — 부팅 시 자동 캘리브레이션 (직진→복귀→360° 회전)
- `wheel_odom_node.py` — 휠 오도메트리 적분 + TF 브로드캐스트
- `pose_to_odom_node.py` — GPS RobotPose → nav_msgs/Odometry 변환
- EKF 센서 퓨전 (10Hz): 휠 오도메트리 + IMU(BNO055) + GPS
- 캘리브레이션 결과 `~/.ros/araseo_calibration.yaml` 저장

### 2-7. araseo_mission_manager — 미션 계획 및 실행
- 토폴로지 그래프 (12 노드, 16 양방향 엣지)
- A* 경로 탐색 (다익스트라 휴리스틱)
- Nav2 NavigateToPose 액션으로 웨이포인트 순차 실행
- SetMission/GetMissionStatus 서비스
- 도착 허용 오차: 0.05m, 목표당 타임아웃: 60초

### 2-8. araseo_hud — 실시간 HUD 오버레이
- 7개 시각 레이어: 진입금지 구역, 주행가능 영역, 차선 표시, 경로 화살표, 웨이포인트, 장애물 박스, 상태 패널
- 알파 블렌딩 (주행가능 0.30, 진입금지 0.18)
- 20Hz 이미지 퍼블리시

### 2-9. gps_field_msgs — GPS 필드 메시지 (빈 플레이스홀더)
- 향후 GPS 서버 호환 메시지 타입 예정

---

## 3. 단독 실행 프로그램

### 3-1. auto_drive.py (ROS2 없이 자율주행)
- 카메라 + Dynamixel SDK 직접 제어
- 검은 도로 영역을 HSV로 마스킹하여 도로 중심 추종
- 커브 보정: 좌커브 시 우측 편향 20%, 우커브 시 좌측 편향 20%
- 도로 폭 기반 속도 조절 (폭 < 100px → 정지, < 200px → 50%)
- MJPEG 웹 스트리밍 + 긴급 정지 버튼 (포트 8089)
- 기본 속도: 30, 최대 조향: 35

### 3-2. lane_detection.py (차선 검출 테스트)
- LAB + CLAHE + 적응형 임계값 기반 차선 검출
- Flask 웹 UI (포트 5000) + 실시간 파라미터 슬라이더
- Picamera2 입력, MJPEG 출력

---

## 4. 시스템 서비스 (자동 시작)

| 서비스 | 설명 | 상태 |
|--------|------|------|
| `pinky-ap.service` | AP 네트워크 (192.168.4.1, SSID: pinky_xxxx, PW: pinkypro) | enabled, active |
| `pinky-boot.service` | 부팅 부저 + LCD 환영 화면 | enabled, active |
| `jupyter.service` | Jupyter Notebook (192.168.4.1:8888, 토큰 없음) | enabled, active |

---

## 5. 하드웨어 구성

| 장치 | 모델 | 인터페이스 | 비고 |
|------|------|-----------|------|
| 모터 × 2 | Dynamixel XL330-M288-T | /dev/ttyAMA4, 1Mbps | ID 1(좌), 2(우), 휠 반지름 32mm, 트랙 90mm |
| 카메라 | Picamera2 | CSI | 640×480 @ 20Hz |
| IMU | BNO055 | I2C 0x28 | 9축, 오일러/쿼터니언 |
| LiDAR | Slamtec RPLiDAR C1 | USB | FOV 90°, 유효 800mm |
| IR × 3 | TCRT5000 | GPIO | 디지털 반사 감지 |
| 초음파 | US-016 | GPIO | 2~400cm ToF |
| 배터리 | 커스텀 ADC | I2C 0x08 | 6.8~7.6V |
| LCD | ILI9341 240×320 | SPI 40MHz | GPIO 27/25/18 (RST/DC/BL) |
| 부저 | PWM | GPIO4 | C5/E5/G5 주파수 |

---

## 6. 데이터 흐름

```
호스트 PC (GPS 카메라)
    │ /pinky_{id}/gps_pos
    ▼
gps_camera_receiver ──► pose_to_odom ──┐
                                       ▼
                                    EKF (10Hz)  ◄── IMU (/imu/data)
                                       ▲
wheel_odom ◄── cmd_vel_safe ───────────┘
    │
    ▼
카메라 ──► lane_follower (20Hz) ──► cmd_vel
                                      │
                    LiDAR/초음파 ──► traffic_law_manager ──► cmd_vel_safe
                                                              │
                                                         Dynamixel 모터
카메라 ──► hud_overlay (20Hz) ──► 시각화 이미지

mission_manager ──► Nav2 ──► 웨이포인트 네비게이션
```

---

## 7. 빌드 상태

- **ROS2 배포판**: Jazzy
- **마지막 빌드**: 2026-04-10
- **빌드 크기**: build/ 21MB, install/ 5.7MB
- **전체 패키지**: 9개 모두 빌드 성공
- **Git**: 미사용 (수동 관리)

---

## 8. 주요 설정 파라미터 요약

| 항목 | 파라미터 | 값 | 단위 |
|------|---------|-----|------|
| 차선 추종 | 기본 속도 | 0.15 | m/s |
| | 최대 조향 | 0.524 (30°) | rad |
| | 전방주시 (직진) | 0.30 | m |
| | 전방주시 (커브) | 0.15 | m |
| | 슬라이딩 윈도우 | 9개, 마진 60px | |
| 교통법 | 정지선 대기 | 2.0 | s |
| | 앞차 추종 거리 | 400 | mm |
| | 충돌 정지 거리 | 200 | mm |
| | LiDAR 감속 거리 | 400 | mm |
| | 초음파 정지 거리 | 150 | mm |
| 위치 추정 | EKF 주파수 | 10 | Hz |
| | 센서 타임아웃 | 0.5 | s |
| 미션 | 도착 허용 오차 | 0.05 | m |
| | 목표당 타임아웃 | 60 | s |
| 도로 | 도로 폭 | 280 | mm |
| | 차선 폭 | 140 | mm |
| | 맵 크기 | 1880×1410 | mm |

---

## 9. 미완료 / 진행 중 항목

- BEV 캘리브레이션 파라미터 최적화 미완 (현재 기본값 사용)
- HSV 임계값 튜닝 진행 중 (실내 조명 변화 대응)
- auto_drive.py 단독 모드로 검은 도로 추종 중 — 커브 안정성 개선 필요
- Fleet 다중 로봇 동시 운용 테스트 미실시
- Nav2 웨이포인트 네비게이션 실제 주행 검증 미완
- gps_field_msgs 패키지 내용 미구현
