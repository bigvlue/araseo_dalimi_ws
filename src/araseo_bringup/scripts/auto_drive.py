#!/usr/bin/env python3
"""
검은 도로 기반 자율주행 + 웹 모니터링
http://<robot_ip>:8089 접속
"""
import sys, time, threading, signal
from collections import deque
sys.path.insert(0, '/home/pinky/pinkylib/sensor/pinkylib')

import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer
from camera import Camera
from battery import Battery
from ultrasonic import Ultrasonic
from dynamixel_sdk import PortHandler, PacketHandler

# ── 설정 ──
IMG_W, IMG_H = 640, 480
STREAM_PORT = 8089
FPS = 15

BASE_SPEED = 30
MAX_TURN = 35
WHEEL_L, WHEEL_R = 1, 2
ADDR_GOAL_VEL = 104
DIR_L, DIR_R = 1, -1

ROAD_S_MAX = 100
ROAD_V_MAX = 90
# 옵션 1: V 우선 검출용 보조 임계값
ROAD_DARK_V = 90       # 이 값 이하면 색 무관 도로로 인식
ROAD_GRAY_S = 60       # 채도 낮으면 (회색조)
ROAD_GRAY_V = 130      # 약간 밝아도 도로로 인식

STEER_Y_TOP = 350
STEER_Y_BOT = 450
LOOKAHEAD_Y = 300

AWB_G = 0.89
YELLOW_LOW = np.array([15, 80, 80])
YELLOW_HIGH = np.array([35, 255, 255])

# BEV (Bird-Eye-View) 변환 — lane_params.yaml 기본값
BEV_SRC = np.float32([[  0, 480], [640, 480], [150, 300], [560, 300]])
BEV_DST = np.float32([[100, 480], [540, 480], [100,   0], [540,   0]])
BEV_M   = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)
# BEV 유효 영역 마스크: 원본 사다리꼴 안쪽만 흰색, 밖은 검정
# 꼭짓점 순서: BL, BR, TR, TL (시계방향, 자기교차 없음)
_bev_src_poly = np.array([BEV_SRC[0], BEV_SRC[1], BEV_SRC[3], BEV_SRC[2]], dtype=np.int32)
_bev_src_mask = np.zeros((480, 640), np.uint8)
cv2.fillConvexPoly(_bev_src_mask, _bev_src_poly, 255)
BEV_VALID = cv2.warpPerspective(_bev_src_mask, BEV_M, (640, 480))
# 경계 가장자리 2px 살짝 안쪽으로 수축 (보간 노이즈 제거)
BEV_VALID = cv2.erode(BEV_VALID, np.ones((3, 3), np.uint8), iterations=2)

# ── 전역 ──
latest_jpeg = None
lock = threading.Lock()
running = True
driving_enabled = False    # 웹에서 START 누르기 전엔 정지 상태
camera_enabled  = True     # 카메라 활성(배터리 절약용 토글)
battery_v = 0.0            # 배터리 전압 (V), pinkylib I2C ADC 기반 (Pinky Pro = 2S LiPo)
battery_pct = 0            # 배터리 잔량 (%)
battery_state = '?'        # CHARGING / DISCHARGING / IDLE / ?
battery_hist = deque(maxlen=60)   # (t, v) 샘플, 최대 60개 (약 2분, 2s 간격)
ultrasonic_dist_m = None          # 초음파 센서 거리 (m), None=읽기 실패

# 런타임 튜닝 가능 파라미터 (웹에서 조정)
params = {
    'weight_near':     0.70,   # 가까운 영역 가중치 (먼 영역 = 1 - 이값)
    'curve_threshold': 15.0,   # 커브 판정 임계 (px, look-near 차이)
    'curve_offset':    0.20,   # 커브 시 차로폭 대비 편향 비율
}


def signal_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def ultrasonic_loop():
    """초음파 센서를 10Hz로 지속 측정하는 백그라운드 스레드"""
    global ultrasonic_dist_m
    try:
        us = Ultrasonic()
        print("[초음파] 초기화 완료 (I2C 0x08)")
    except Exception as e:
        print(f"[경고] 초음파 센서 초기화 실패: {e}")
        return
    while running:
        try:
            d = us.get_dist()
            if d is not None and d >= 0:
                ultrasonic_dist_m = d
            else:
                ultrasonic_dist_m = None
        except Exception:
            ultrasonic_dist_m = None
        time.sleep(0.1)
    us.close()


def drive_loop():
    global latest_jpeg, running

    def open_camera():
        c = Camera()
        c.start(width=IMG_W, height=IMG_H)
        time.sleep(1)
        return c

    cam = open_camera()

    port = PortHandler('/dev/ttyAMA4')
    port.openPort()
    port.setBaudRate(1000000)
    ph = PacketHandler(2.0)
    for mid in [WHEEL_L, WHEEL_R]:
        ph.write1ByteTxRx(port, mid, 64, 1)

    def set_motor(vx, vz):
        left = int((vx + vz) * DIR_L)
        right = int((vx - vz) * DIR_R)
        left = max(-80, min(80, left))
        right = max(-80, min(80, right))
        ph.write4ByteTxRx(port, WHEEL_L, ADDR_GOAL_VEL, left & 0xFFFFFFFF)
        ph.write4ByteTxRx(port, WHEEL_R, ADDR_GOAL_VEL, right & 0xFFFFFFFF)

    def stop_motor():
        ph.write4ByteTxRx(port, WHEEL_L, ADDR_GOAL_VEL, 0)
        ph.write4ByteTxRx(port, WHEEL_R, ADDR_GOAL_VEL, 0)

    # Pinky 배터리 (I2C ADC, 2S LiPo: 만충 7.6V / 방전 6.8V)
    try:
        bat = Battery()
    except Exception as e:
        print(f"[경고] Battery 초기화 실패: {e}")
        bat = None

    def read_battery():
        """pinkylib I2C ADC로 배터리 전압/잔량 읽기 + 충전 상태 추정"""
        global battery_v, battery_pct, battery_state
        if bat is None:
            return
        try:
            v = bat.get_voltage()
            if v is None or not (4.0 <= v <= 10.0):
                return
            battery_v = float(v)
            # 2S LiPo 기준 (7.6V=100%, 6.8V=0%)
            pct = (v - 6.8) / (7.6 - 6.8) * 100.0
            battery_pct = int(max(0, min(100, pct)))
            # 이력 추가 후 선형 기울기(V/s)로 충전/방전 판정
            now = time.monotonic()
            battery_hist.append((now, battery_v))
            if len(battery_hist) >= 10 and (battery_hist[-1][0] - battery_hist[0][0]) >= 30:
                xs = [t for t, _ in battery_hist]
                ys = [vv for _, vv in battery_hist]
                n = len(xs)
                mx = sum(xs) / n; my = sum(ys) / n
                num = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
                den = sum((xs[i] - mx) ** 2 for i in range(n)) or 1e-9
                slope = num / den   # V/s
                if   slope >  0.0005: battery_state = 'CHARGING'
                elif slope < -0.0005: battery_state = 'DISCHARGING'
                else:                 battery_state = 'IDLE'
        except Exception:
            pass

    read_battery()
    print(f"Autonomous driving started (battery: {battery_v:.2f}V, {battery_pct}%)")
    frame_count = 0
    cam_active = True   # 실제 카메라 상태 (camera_enabled와 동기화)

    def make_camera_off_jpeg():
        """CAM OFF 상태 정적 이미지 (배터리 정보 포함)"""
        img = np.zeros((IMG_H, IMG_W, 3), np.uint8)
        img[:] = (30, 30, 30)
        cv2.putText(img, 'CAMERA OFF', (IMG_W//2 - 170, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (100, 180, 255), 3)
        cv2.putText(img, '(battery saving mode)', (IMG_W//2 - 170, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        if battery_v > 0:
            if battery_state == 'CHARGING': col = (255, 200, 0)
            elif battery_v >= 7.3: col = (0, 255, 0)
            elif battery_v >= 7.0: col = (0, 255, 255)
            else: col = (0, 0, 255)
            tag = {'CHARGING': 'CHG', 'DISCHARGING': 'DIS',
                   'IDLE': 'IDLE'}.get(battery_state, '?')
            cv2.putText(img, f'BAT: {battery_v:.2f}V ({battery_pct}%) {tag}',
                        (IMG_W//2 - 200, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
        _, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg.tobytes()

    try:
        while running:
            # ── 카메라 On/Off 토글 처리 ──
            if camera_enabled and not cam_active:
                try:
                    cam = open_camera()   # Picamera2 재생성 필요 (close된 객체 재사용 불가)
                    cam_active = True
                    print("[CAM] 켜짐")
                except Exception as e:
                    print(f"[CAM] 재시작 실패: {e}")
                    time.sleep(1.0)
                    continue
            if not camera_enabled and cam_active:
                stop_motor()
                try:
                    cam.close()
                except Exception:
                    pass
                cam_active = False
                print("[CAM] 꺼짐 (배터리 절약)")
            # 카메라 꺼져있으면 배터리 읽기 + CAM OFF 이미지만
            if not cam_active:
                read_battery()
                with lock:
                    latest_jpeg = make_camera_off_jpeg()
                time.sleep(1.0)
                continue

            frame = cam.get_frame()
            r, g, b = cv2.split(frame)
            g = np.clip(g.astype(np.float32) * AWB_G, 0, 255).astype(np.uint8)
            frame = cv2.merge([r, g, b])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 도로 마스크 (V 우선): 어두우면 색 무관 도로 OR 회색조면 도로
            road_dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, ROAD_DARK_V]))
            road_gray = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, ROAD_GRAY_S, ROAD_GRAY_V]))
            road = cv2.bitwise_or(road_dark, road_gray)
            # 노란선/흰선 등 밝은 영역은 도로에서 제외
            boundary = cv2.bitwise_not(road)

            # 조향 ROI에서 가장 큰 연속 검은 구간(=도로) 찾기
            def find_road_center(roi_mask):
                """각 행에서 가장 넓은 연속 검은 구간을 찾아 중앙/폭/좌우 반환
                반환: (cx, width, left_x, right_x)
                """
                h, w = roi_mask.shape
                s_cx = s_w = s_l = s_r = 0.0
                count = 0
                for y in range(0, h, 3):  # 3행 간격 샘플링
                    row = roi_mask[y, :]
                    # 연속 구간 찾기
                    runs = []
                    start = -1
                    for x in range(w):
                        if row[x] > 0 and start < 0:
                            start = x
                        elif row[x] == 0 and start >= 0:
                            runs.append((start, x - 1))
                            start = -1
                    if start >= 0:
                        runs.append((start, w - 1))
                    if runs:
                        # 가장 넓은 구간
                        best = max(runs, key=lambda r: r[1] - r[0])
                        s_cx += (best[0] + best[1]) / 2.0
                        s_w  += best[1] - best[0]
                        s_l  += best[0]
                        s_r  += best[1]
                        count += 1
                if count > 0:
                    return (s_cx / count, s_w / count, s_l / count, s_r / count)
                return (w / 2.0, 0.0, 0.0, w - 1.0)

            steer_roi = road[STEER_Y_TOP:STEER_Y_BOT, :]
            road_center, road_width, road_left_x, road_right_x = find_road_center(steer_roi)

            # 전방 주시점
            look_roi = road[LOOKAHEAD_Y-20:LOOKAHEAD_Y+20, :]
            look_center, _, _, _ = find_road_center(look_roi)

            # 조향 계산 (런타임 튜닝 파라미터 사용)
            img_center = IMG_W / 2
            wn = params['weight_near']
            ct = params['curve_threshold']
            co = params['curve_offset']
            # 가까운 곳(wn) + 먼 곳(1-wn) 가중 평균
            raw_target = road_center * wn + look_center * (1.0 - wn)

            # 커브 보정: 좌커브=우측 편향, 우커브=좌측 편향
            curve_dir = look_center - road_center  # 양수=우커브, 음수=좌커브
            mode = "STRAIGHT"
            if abs(curve_dir) > ct:
                if curve_dir < 0:
                    target = raw_target + road_width * co
                    mode = "CURVE-L"
                else:
                    target = raw_target - road_width * co
                    mode = "CURVE-R"
            else:
                target = raw_target

            error = (target - img_center) / (IMG_W / 2)  # -1 ~ +1

            # 도로 폭이 좁으면 (벽 가까이) 속도 줄이기
            speed = BASE_SPEED
            if road_width < 100:
                speed = 0  # 도로를 거의 못 찾으면 정지
            elif road_width < 200:
                speed = int(BASE_SPEED * 0.5)

            turn = int(error * MAX_TURN)

            # 웹에서 START 누르기 전엔 모터 정지 (영상은 계속 처리)
            if driving_enabled:
                set_motor(speed, turn)
            else:
                set_motor(0, 0)

            # 시각화 — 검은 도로만 원본, 나머지 전부 마젠타
            disp = frame.copy()
            # 노란 중앙선 별도 감지
            yellow = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
            white_boundary = cv2.bitwise_and(boundary, cv2.bitwise_not(yellow))
            disp[white_boundary > 0] = [255, 0, 255]
            disp[yellow > 0] = [0, 255, 255]

            # 조향 ROI 표시
            cv2.rectangle(disp, (0, STEER_Y_TOP), (IMG_W, STEER_Y_BOT), (0, 255, 0), 1)
            # 도로 중앙 표시
            cx = int(target)
            cv2.line(disp, (cx, STEER_Y_TOP), (cx, STEER_Y_BOT), (0, 255, 0), 2)
            cv2.line(disp, (IMG_W//2, STEER_Y_TOP), (IMG_W//2, STEER_Y_BOT), (255, 255, 0), 1)
            # 전방 주시점
            cv2.circle(disp, (int(look_center), LOOKAHEAD_Y), 8, (255, 0, 0), 2)

            # 상태 텍스트
            roi_total = (STEER_Y_BOT - STEER_Y_TOP) * IMG_W
            roi_road = np.count_nonzero(steer_roi)
            road_pct = 100.0 * roi_road / roi_total if roi_total > 0 else 0

            if not driving_enabled:
                status = "PAUSE"
            else:
                status = "DRIVE" if speed > 0 else "STOP"
            cv2.putText(disp, f'{status} {mode} spd={speed} turn={turn} w={road_width:.0f} road={road_pct:.0f}%',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # 배터리 표시 (우상단, 2S LiPo 기준) + 충전/방전 상태
            if battery_state == 'CHARGING':
                bat_color = (255, 200, 0)    # BGR 파랑/시안: 충전
            elif battery_v >= 7.3:
                bat_color = (0, 255, 0)
            elif battery_v >= 7.0:
                bat_color = (0, 255, 255)
            else:
                bat_color = (0, 0, 255)
            if battery_v > 0:
                tag = {'CHARGING': 'CHG', 'DISCHARGING': 'DIS',
                       'IDLE': 'IDLE'}.get(battery_state, '?')
                bat_text = f'BAT: {battery_v:.2f}V ({battery_pct}%) {tag}'
            else:
                bat_text = 'BAT: --'
            cv2.rectangle(disp, (IMG_W - 305, 5), (IMG_W - 5, 35), (0, 0, 0), -1)
            cv2.putText(disp, bat_text, (IMG_W - 300, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, bat_color, 2)
            # 초음파 거리 표시 (배터리 아래)
            if ultrasonic_dist_m is not None:
                dist_cm = ultrasonic_dist_m * 100.0
                dist_color = (0, 0, 255) if dist_cm < 20 else (0, 255, 255) if dist_cm < 50 else (0, 255, 0)
                dist_text = f'DIST: {dist_cm:.1f} cm'
            else:
                dist_color = (128, 128, 128)
                dist_text = 'DIST: --'
            cv2.rectangle(disp, (IMG_W - 220, 38), (IMG_W - 5, 62), (0, 0, 0), -1)
            cv2.putText(disp, dist_text, (IMG_W - 215, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, dist_color, 2)
            # 원본 기반 L/R/W (교차로 감지용)
            cv2.putText(disp,
                        f'L={road_left_x:.0f} R={road_right_x:.0f} W={road_width:.0f}',
                        (10, IMG_H - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            # 좌/우 extent 세로선 표시 (조향 ROI 내)
            cv2.line(disp, (int(road_left_x),  STEER_Y_TOP), (int(road_left_x),  STEER_Y_BOT),
                     (255, 255, 0), 2)
            cv2.line(disp, (int(road_right_x), STEER_Y_TOP), (int(road_right_x), STEER_Y_BOT),
                     (0, 255, 255), 2)

            # ── 디버그: 우측 하단 영역 H/S/V 분석 ──
            BR_X1, BR_Y1, BR_X2, BR_Y2 = 480, 320, 640, 480
            br_hsv = hsv[BR_Y1:BR_Y2, BR_X1:BR_X2]
            br_h, br_s, br_v = cv2.split(br_hsv)
            br_road = road[BR_Y1:BR_Y2, BR_X1:BR_X2]
            br_total = br_road.size
            br_road_cnt = np.count_nonzero(br_road)
            br_road_pct = 100.0 * br_road_cnt / br_total if br_total > 0 else 0
            # 임계값 초과 픽셀 비율
            s_over = 100.0 * np.count_nonzero(br_s > ROAD_S_MAX) / br_total
            v_over = 100.0 * np.count_nonzero(br_v > ROAD_V_MAX) / br_total
            # ROI 표시 (시안색)
            cv2.rectangle(disp, (BR_X1, BR_Y1), (BR_X2-1, BR_Y2-1), (255, 255, 0), 2)
            # 텍스트 (배경 어둡게)
            cv2.rectangle(disp, (5, 35), (470, 95), (0, 0, 0), -1)
            cv2.putText(disp, f'BR H avg/max: {br_h.mean():.0f}/{br_h.max():.0f}',
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.putText(disp, f'BR S avg/max: {br_s.mean():.0f}/{br_s.max():.0f}  (limit<={ROAD_S_MAX}, over={s_over:.0f}%)',
                        (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.putText(disp, f'BR V avg/max: {br_v.mean():.0f}/{br_v.max():.0f}  (limit<={ROAD_V_MAX}, over={v_over:.0f}%)  road={br_road_pct:.0f}%',
                        (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

            # ── BEV 변환 (시각화 전용, 조향에는 미사용) ──
            bev = cv2.warpPerspective(disp, BEV_M, (IMG_W, IMG_H))

            # ── BEV 도로 마스크 + 좌/우 extent 측정 (교차로 감지 단계 1) ──
            bev_road = cv2.warpPerspective(road, BEV_M, (IMG_W, IMG_H))
            # BEV 유효 영역만 유지 (사다리꼴 밖의 검은 테두리 제외)
            bev_road = cv2.bitwise_and(bev_road, BEV_VALID)
            BEV_ROI_TOP, BEV_ROI_BOT = 200, 440
            bev_roi = bev_road[BEV_ROI_TOP:BEV_ROI_BOT, :]
            left_xs = []
            right_xs = []
            for ry in range(0, bev_roi.shape[0], 4):
                idx = np.flatnonzero(bev_roi[ry, :])
                if idx.size > 5:
                    left_xs.append(int(idx.min()))
                    right_xs.append(int(idx.max()))
            if left_xs:
                bev_left  = int(np.mean(left_xs))
                bev_right = int(np.mean(right_xs))
                bev_w     = bev_right - bev_left
            else:
                bev_left = bev_right = bev_w = 0
            # 시각화: BEV에 좌/우 extent 라인 + ROI 박스
            cv2.rectangle(bev, (0, BEV_ROI_TOP), (IMG_W-1, BEV_ROI_BOT-1), (80, 80, 80), 1)
            if bev_w > 0:
                cv2.line(bev, (bev_left,  BEV_ROI_TOP), (bev_left,  BEV_ROI_BOT), (255, 255, 0), 2)
                cv2.line(bev, (bev_right, BEV_ROI_TOP), (bev_right, BEV_ROI_BOT), (0, 255, 255), 2)
            cv2.rectangle(bev, (5, 5), (290, 30), (0, 0, 0), -1)
            cv2.putText(bev, f'L={bev_left} R={bev_right} W={bev_w}',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            # BEV에 src 사다리꼴(원본에서 변환된 영역) 표시
            for i in range(4):
                p1 = tuple(BEV_SRC[i].astype(int))
                p2 = tuple(BEV_SRC[(i+1) % 4].astype(int))
                cv2.line(disp, p1, p2, (0, 255, 255), 1)
            cv2.putText(disp, 'ORIGINAL', (10, IMG_H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(bev, 'BEV', (10, IMG_H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            # 좌우 결합 (원본 해상도 유지: 1280×480)
            combined = np.hstack([disp, bev])

            _, jpeg = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with lock:
                latest_jpeg = jpeg.tobytes()

            frame_count += 1
            if frame_count % 30 == 0:
                print(f"[{frame_count}] spd={speed} turn={turn} err={error:.2f} width={road_width:.0f} bat={battery_v:.1f}V")
                read_battery()   # 약 2초마다 갱신

            time.sleep(1.0 / FPS)

    finally:
        print("Stopping motors...")
        stop_motor()
        for mid in [WHEEL_L, WHEEL_R]:
            ph.write1ByteTxRx(port, mid, 64, 0)
        port.closePort()
        cam.close()
        print("Stopped.")


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global running, driving_enabled, camera_enabled
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write('''<html><head><meta charset="utf-8"><title>Auto Drive</title>
<style>body{background:#111;margin:0;display:flex;flex-direction:column;align-items:center;color:#fff;font-family:sans-serif}
img{width:98vw;max-width:1600px;height:auto;image-rendering:auto}
.row{display:flex;gap:10px;margin:10px}
button{padding:18px 36px;font-size:22px;color:#fff;border:none;border-radius:10px;cursor:pointer;font-weight:bold}
.start{background:#2ecc40}
.pause{background:#ff851b}
.stop{background:#ff4136}
.cam{background:#0074d9}
.camoff{background:#555}
#state{font-size:24px;margin:8px;font-weight:bold}
.params{background:#222;padding:12px 20px;border-radius:8px;margin:10px;min-width:560px}
.param{display:flex;align-items:center;gap:10px;margin:8px 0}
.param label{flex:1;font-size:16px}
.param input[type=range]{flex:2}
.param .v{width:60px;text-align:right;font-family:monospace;font-size:16px;color:#ffe066}
</style></head><body>
<img src="/stream">
<div id="state">State: loading...</div>
<div id="battery" style="font-size:22px;margin:4px;font-weight:bold">BAT: --</div>
<div id="ultrasonic" style="font-size:22px;margin:4px;font-weight:bold">DIST: --</div>
<div class="row">
  <button class="start" onclick="cmd('start')">START</button>
  <button class="pause" onclick="cmd('pause')">PAUSE</button>
  <button id="btn_cam" class="cam" onclick="toggleCam()">CAM ON</button>
  <button class="stop" onclick="if(confirm('Stop program?'))cmd('stop')">EMERGENCY</button>
</div>
<div class="params">
  <div class="param">
    <label>weight_near (가까운 영역 가중치)</label>
    <input id="p_weight_near" type="range" min="0" max="1" step="0.05"
           oninput="setp('weight_near', this.value)">
    <span class="v" id="v_weight_near">-</span>
  </div>
  <div class="param">
    <label>curve_threshold (커브 판정 px)</label>
    <input id="p_curve_threshold" type="range" min="0" max="50" step="1"
           oninput="setp('curve_threshold', this.value)">
    <span class="v" id="v_curve_threshold">-</span>
  </div>
  <div class="param">
    <label>curve_offset (커브 편향 비율)</label>
    <input id="p_curve_offset" type="range" min="0" max="0.5" step="0.01"
           oninput="setp('curve_offset', this.value)">
    <span class="v" id="v_curve_offset">-</span>
  </div>
</div>
<script>
function cmd(c){fetch('/'+c).then(r=>r.text()).then(refresh)}
function toggleCam(){
  fetch('/cam_state').then(r=>r.text()).then(s=>{
    fetch(s==='ON' ? '/cam_off' : '/cam_on').then(refresh);
  });
}
function refresh(){fetch('/state').then(r=>r.text()).then(t=>{
  document.getElementById('state').textContent='State: '+t;
  document.getElementById('state').style.color = (t==='DRIVING')?'#2ecc40':'#ff851b';
})
fetch('/cam_state').then(r=>r.text()).then(s=>{
  const b = document.getElementById('btn_cam');
  b.textContent = 'CAM ' + s;
  b.className = s==='ON' ? 'cam' : 'camoff';
})
fetch('/battery').then(r=>r.text()).then(v=>{
  const [vs, ps, st] = v.split(' ');
  const f = parseFloat(vs);
  const p = parseInt(ps);
  const icon = st==='CHARGING'?' ⚡ 충전중' : st==='DISCHARGING'?' ▼ 방전' : st==='IDLE'?' ■ 유지' : '';
  const el = document.getElementById('battery');
  el.textContent = 'BAT: ' + (f>0 ? f.toFixed(2)+'V ('+p+'%)'+icon : '--');
  el.style.color = st==='CHARGING' ? '#4db5ff' : f>=7.3?'#2ecc40' : f>=7.0?'#ffe066' : '#ff4136';
})
fetch('/ultrasonic').then(r=>r.text()).then(d=>{
  const el = document.getElementById('ultrasonic');
  if(d==='--'){el.textContent='DIST: --'; el.style.color='#888'; return;}
  const cm = parseFloat(d);
  el.textContent = 'DIST: ' + cm.toFixed(1) + ' cm';
  el.style.color = cm<20?'#ff4136' : cm<50?'#ffe066' : '#2ecc40';
})}
function setp(k, v){
  document.getElementById('v_'+k).textContent = (+v).toFixed(2);
  fetch('/set?key='+k+'&val='+v);
}
function loadp(){
  fetch('/params').then(r=>r.json()).then(p=>{
    for(const k in p){
      const inp = document.getElementById('p_'+k);
      const sp = document.getElementById('v_'+k);
      if(inp) inp.value = p[k];
      if(sp) sp.textContent = (+p[k]).toFixed(2);
    }
  });
}
setInterval(refresh,1000); refresh(); loadp();
</script>
</body></html>'''.encode('utf-8'))
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with lock:
                    jpeg = latest_jpeg
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
                    time.sleep(1.0 / FPS)
                except BrokenPipeError:
                    break
        elif self.path == '/stop':
            running = False
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'stopped')
        elif self.path == '/start':
            if camera_enabled:
                driving_enabled = True
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'started')
            else:
                self.send_response(409)
                self.end_headers()
                self.wfile.write(b'camera off')
        elif self.path == '/pause':
            driving_enabled = False
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'paused')
        elif self.path == '/state':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'DRIVING' if driving_enabled else b'PAUSED')
        elif self.path == '/cam_on':
            camera_enabled = True
            self.send_response(200); self.end_headers(); self.wfile.write(b'cam_on')
        elif self.path == '/cam_off':
            camera_enabled = False
            driving_enabled = False   # 카메라 꺼지면 반드시 정지
            self.send_response(200); self.end_headers(); self.wfile.write(b'cam_off')
        elif self.path == '/cam_state':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'ON' if camera_enabled else b'OFF')
        elif self.path == '/battery':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(f'{battery_v:.2f} {battery_pct} {battery_state}'.encode('utf-8'))
        elif self.path == '/ultrasonic':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            if ultrasonic_dist_m is not None:
                self.wfile.write(f'{ultrasonic_dist_m * 100.0:.1f}'.encode('utf-8'))
            else:
                self.wfile.write(b'--')
        elif self.path == '/params':
            import json
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(params).encode('utf-8'))
        elif self.path.startswith('/set?'):
            from urllib.parse import urlparse, parse_qs
            qs = parse_qs(urlparse(self.path).query)
            key = qs.get('key', [None])[0]
            val = qs.get('val', [None])[0]
            ok = False
            if key in params and val is not None:
                try:
                    params[key] = float(val)
                    ok = True
                except ValueError:
                    pass
            self.send_response(200 if ok else 400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'ok' if ok else b'bad')
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


if __name__ == '__main__':
    threading.Thread(target=ultrasonic_loop, daemon=True).start()
    t = threading.Thread(target=drive_loop, daemon=True)
    t.start()
    print(f'Monitor: http://0.0.0.0:{STREAM_PORT}')
    ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), MJPEGHandler).serve_forever()
