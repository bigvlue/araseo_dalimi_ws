#!/usr/bin/env python3
"""
검은 도로 기반 자율주행 + 웹 모니터링
http://<robot_ip>:8089 접속
"""
import sys, time, threading, signal
sys.path.insert(0, '/home/pinky/pinkylib/sensor/pinkylib')

import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer
from camera import Camera
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

# ── 전역 ──
latest_jpeg = None
lock = threading.Lock()
running = True
driving_enabled = False    # 웹에서 START 누르기 전엔 정지 상태


def signal_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def drive_loop():
    global latest_jpeg, running

    cam = Camera()
    cam.start(width=IMG_W, height=IMG_H)
    time.sleep(1)

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

    print("Autonomous driving started")
    frame_count = 0

    try:
        while running:
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
                """각 행에서 가장 넓은 연속 검은 구간을 찾아 중앙 반환"""
                h, w = roi_mask.shape
                best_cx_sum = 0.0
                best_width_sum = 0.0
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
                        cx = (best[0] + best[1]) / 2.0
                        bw = best[1] - best[0]
                        best_cx_sum += cx
                        best_width_sum += bw
                        count += 1
                if count > 0:
                    return best_cx_sum / count, best_width_sum / count
                return w / 2.0, 0.0

            steer_roi = road[STEER_Y_TOP:STEER_Y_BOT, :]
            road_center, road_width = find_road_center(steer_roi)

            # 전방 주시점
            look_roi = road[LOOKAHEAD_Y-20:LOOKAHEAD_Y+20, :]
            look_center, _ = find_road_center(look_roi)

            # 조향 계산
            img_center = IMG_W / 2
            # 가까운 곳(70%) + 먼 곳(30%) 가중 평균
            raw_target = road_center * 0.7 + look_center * 0.3

            # 커브 보정: 좌커브 70:30(우측 편향), 우커브 30:70(좌측 편향)
            curve_dir = look_center - road_center  # 양수=우커브, 음수=좌커브
            mode = "STRAIGHT"
            if abs(curve_dir) > 15:
                if curve_dir < 0:
                    # 좌커브: 70:30 → 우측으로 20% 편향
                    target = raw_target + road_width * 0.20
                    mode = "CURVE-L"
                else:
                    # 우커브: 30:70 → 좌측으로 20% 편향
                    target = raw_target - road_width * 0.20
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

            h2 = IMG_H // 2
            w2 = IMG_W // 2
            small = cv2.resize(disp, (w2*2, h2*2))

            _, jpeg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_jpeg = jpeg.tobytes()

            frame_count += 1
            if frame_count % 30 == 0:
                print(f"[{frame_count}] spd={speed} turn={turn} err={error:.2f} width={road_width:.0f}")

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
        global running, driving_enabled
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write('''<html><head><meta charset="utf-8"><title>Auto Drive</title>
<style>body{background:#111;margin:0;display:flex;flex-direction:column;align-items:center;color:#fff;font-family:sans-serif}
img{max-width:100%;height:auto}
.row{display:flex;gap:10px;margin:10px}
button{padding:18px 36px;font-size:22px;color:#fff;border:none;border-radius:10px;cursor:pointer;font-weight:bold}
.start{background:#2ecc40}
.pause{background:#ff851b}
.stop{background:#ff4136}
#state{font-size:24px;margin:8px;font-weight:bold}
</style></head><body>
<img src="/stream">
<div id="state">State: loading...</div>
<div class="row">
  <button class="start" onclick="cmd('start')">START</button>
  <button class="pause" onclick="cmd('pause')">PAUSE</button>
  <button class="stop" onclick="if(confirm('Stop program?'))cmd('stop')">EMERGENCY</button>
</div>
<script>
function cmd(c){fetch('/'+c).then(r=>r.text()).then(refresh)}
function refresh(){fetch('/state').then(r=>r.text()).then(t=>{
  document.getElementById('state').textContent='State: '+t;
  document.getElementById('state').style.color = (t==='DRIVING')?'#2ecc40':'#ff851b';
})}
setInterval(refresh,1000); refresh();
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
            driving_enabled = True
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'started')
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
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


if __name__ == '__main__':
    t = threading.Thread(target=drive_loop, daemon=True)
    t.start()
    print(f'Monitor: http://0.0.0.0:{STREAM_PORT}')
    ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), MJPEGHandler).serve_forever()
