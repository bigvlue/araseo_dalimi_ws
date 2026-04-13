#!/usr/bin/env python3
"""
카메라 스트림 + 웹 텔레옵 통합
http://<robot_ip>:8089 접속 → WASD 키로 조종 + 실시간 차선 검출 뷰
"""
import sys, time, threading, json
sys.path.insert(0, '/home/pinky/pinkylib/sensor/pinkylib')

import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from camera import Camera

# Dynamixel 직접 제어
try:
    from dynamixel_sdk import PortHandler, PacketHandler
    DXL_OK = True
except ImportError:
    DXL_OK = False

# ── 설정 ──
IMG_W, IMG_H = 640, 480
STREAM_PORT  = 8089
FPS          = 10

BEV_SRC = np.float32([[11,440],[628,440],[209,280],[422,280]])
BEV_DST = np.float32([[80,480],[560,480],[80,0],[560,0]])

YELLOW_LOW  = np.array([15, 80, 80])
YELLOW_HIGH = np.array([35, 255, 255])
AWB_G = 0.89

# 모터 설정
BASE_SPEED = 40       # Dynamixel raw velocity
TURN_SPEED = 25
WHEEL_L, WHEEL_R = 1, 2
ADDR_GOAL_VEL = 104
DIR_L, DIR_R = 1, -1

# ── 전역 ──
latest_jpeg = None
lock = threading.Lock()
motor_cmd = {'vx': 0, 'vz': 0}  # linear, angular
motor_lock = threading.Lock()


def motor_loop():
    if not DXL_OK:
        print("Dynamixel SDK not available")
        return
    port = PortHandler('/dev/ttyAMA4')
    port.openPort()
    port.setBaudRate(1000000)
    ph = PacketHandler(2.0)
    # Torque on
    for mid in [WHEEL_L, WHEEL_R]:
        ph.write1ByteTxRx(port, mid, 64, 1)
    print("Motor ready")

    last_cmd_time = time.time()
    while True:
        with motor_lock:
            vx = motor_cmd['vx']
            vz = motor_cmd['vz']

        left  = int((vx + vz) * DIR_L)
        right = int((vx - vz) * DIR_R)

        # Clamp
        left  = max(-80, min(80, left))
        right = max(-80, min(80, right))

        ph.write4ByteTxRx(port, WHEEL_L, ADDR_GOAL_VEL, left & 0xFFFFFFFF)
        ph.write4ByteTxRx(port, WHEEL_R, ADDR_GOAL_VEL, right & 0xFFFFFFFF)

        time.sleep(0.05)  # 20Hz


def process_loop():
    global latest_jpeg
    cam = Camera()
    cam.start(width=IMG_W, height=IMG_H)
    time.sleep(1)

    M = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)

    while True:
        frame = cam.get_frame()
        r, g, b = cv2.split(frame)
        g = np.clip(g.astype(np.float32) * AWB_G, 0, 255).astype(np.uint8)
        frame = cv2.merge([r, g, b])

        hsv_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_raw = cv2.inRange(hsv_raw, YELLOW_LOW, YELLOW_HIGH)
        road_raw = cv2.inRange(hsv_raw, np.array([0,0,0]), np.array([180,100,90]))
        boundary_raw = cv2.bitwise_not(road_raw)
        white_raw = cv2.bitwise_and(boundary_raw, cv2.bitwise_not(yellow_raw))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        white_raw = cv2.morphologyEx(white_raw, cv2.MORPH_OPEN, kernel)

        bev = cv2.warpPerspective(frame, M, (IMG_W, IMG_H))
        yellow = cv2.warpPerspective(yellow_raw, M, (IMG_W, IMG_H))
        white = cv2.warpPerspective(white_raw, M, (IMG_W, IMG_H))
        _, yellow = cv2.threshold(yellow, 127, 255, cv2.THRESH_BINARY)
        _, white = cv2.threshold(white, 127, 255, cv2.THRESH_BINARY)

        overlay = bev.copy()
        overlay[yellow > 0] = [0, 255, 255]
        overlay[white > 0]  = [255, 0, 255]

        raw_disp = frame.copy()
        raw_disp[yellow_raw > 0] = [0, 255, 255]
        raw_disp[white_raw > 0]  = [255, 0, 255]
        order = np.array([BEV_SRC[0], BEV_SRC[2], BEV_SRC[3], BEV_SRC[1]], dtype=np.int32)
        cv2.polylines(raw_disp, [order], True, (0, 255, 0), 2)

        h2, w2 = IMG_H // 2, IMG_W // 2
        r1 = cv2.resize(raw_disp, (w2, h2))
        r2 = cv2.resize(bev,      (w2, h2))
        r3 = cv2.resize(overlay,  (w2, h2))

        cv2.putText(r1, 'RAW', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(r2, 'BEV', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(r3, 'LANE', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # 속도 표시
        with motor_lock:
            vx, vz = motor_cmd['vx'], motor_cmd['vz']
        cv2.putText(r1, f'V:{vx} T:{vz}', (10, h2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        combined = np.hstack([r1, r2, r3])
        _, jpeg = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock:
            latest_jpeg = jpeg.tobytes()

        time.sleep(1.0 / FPS)


HTML_PAGE = '''<html><head><title>Pinky Teleop</title>
<style>
body{background:#111;color:#eee;margin:0;font-family:monospace;display:flex;flex-direction:column;align-items:center}
img{max-width:100%;height:auto;margin-top:5px}
.controls{display:flex;gap:5px;margin:10px;flex-wrap:wrap;justify-content:center}
.controls button{width:70px;height:50px;font-size:18px;font-weight:bold;background:#333;color:#0f0;border:1px solid #0f0;border-radius:5px;cursor:pointer}
.controls button:active,.controls button.active{background:#0f0;color:#000}
.info{font-size:12px;color:#888;margin:5px}
</style></head>
<body>
<img id="stream" src="/stream">
<div class="controls">
  <button id="bQ" onclick="send('q')">&#8598;Q</button>
  <button id="bW" onclick="send('w')">&#9650;W</button>
  <button id="bE" onclick="send('e')">&#8599;E</button>
  <br>
  <button id="bA" onclick="send('a')">&#9664;A</button>
  <button id="bS" onclick="send('s')">&#9632;S</button>
  <button id="bD" onclick="send('d')">&#9654;D</button>
</div>
<div class="info">WASD/QE = drive | key release = stop</div>
<script>
let keys={};
let interval=null;
function send(k){fetch('/cmd?k='+k)}
function update(){
  if(keys['w']&&keys['a'])send('q');
  else if(keys['w']&&keys['d'])send('e');
  else if(keys['w'])send('w');
  else if(keys['a'])send('a');
  else if(keys['d'])send('d');
  else send('s');
}
const m={w:'w',a:'a',s:'s',d:'d',q:'q',e:'e',' ':'s',
         ArrowUp:'w',ArrowLeft:'a',ArrowDown:'s',ArrowRight:'d'};
document.addEventListener('keydown',function(e){
  let k=m[e.key];if(!k)return;e.preventDefault();
  if(!keys[k]){keys[k]=true;update()}
  let b=document.getElementById('b'+k.toUpperCase());
  if(b)b.classList.add('active');
});
document.addEventListener('keyup',function(e){
  let k=m[e.key];if(!k)return;
  keys[k]=false;update();
  let b=document.getElementById('b'+k.toUpperCase());
  if(b)b.classList.remove('active');
});
window.addEventListener('blur',function(){keys={};send('s')});
// 100ms 간격으로 연속 전송 (키 누르는 동안)
setInterval(function(){
  let any=Object.values(keys).some(v=>v);
  if(any)update();
},100);
</script>
</body></html>'''


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
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
        elif self.path.startswith('/cmd'):
            key = self.path.split('k=')[-1] if 'k=' in self.path else ''
            with motor_lock:
                if key == 'w':
                    motor_cmd['vx'] = BASE_SPEED
                    motor_cmd['vz'] = 0
                elif key == 's':
                    motor_cmd['vx'] = 0
                    motor_cmd['vz'] = 0
                elif key == 'a':
                    motor_cmd['vx'] = 0
                    motor_cmd['vz'] = -TURN_SPEED
                elif key == 'd':
                    motor_cmd['vx'] = 0
                    motor_cmd['vz'] = TURN_SPEED
                elif key == 'q':
                    motor_cmd['vx'] = BASE_SPEED
                    motor_cmd['vz'] = -TURN_SPEED
                elif key == 'e':
                    motor_cmd['vx'] = BASE_SPEED
                    motor_cmd['vz'] = TURN_SPEED
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'ok')
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


if __name__ == '__main__':
    threading.Thread(target=process_loop, daemon=True).start()
    threading.Thread(target=motor_loop, daemon=True).start()
    print(f'Teleop stream: http://0.0.0.0:{STREAM_PORT}')
    print('WASD = drive, S = stop')
    HTTPServer(('0.0.0.0', STREAM_PORT), Handler).serve_forever()
