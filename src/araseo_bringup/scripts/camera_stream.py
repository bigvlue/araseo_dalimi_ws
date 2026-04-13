#!/usr/bin/env python3
"""
카메라 원본 + BEV + 차선 오버레이 실시간 웹 스트림
http://<robot_ip>:8089 접속
"""
import sys, time, threading
sys.path.insert(0, '/home/pinky/pinkylib/sensor/pinkylib')

import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from camera import Camera

# ── 설정 ──
IMG_W, IMG_H = 640, 480
STREAM_PORT  = 8089
FPS          = 10

# BEV 파라미터 (현재 lane_follower 값)
BEV_SRC = np.float32([[11,440],[628,440],[209,280],[422,280]])
BEV_DST = np.float32([[80,480],[560,480],[80,0],[560,0]])

# HSV 범위
YELLOW_LOW  = np.array([15, 80, 80])
YELLOW_HIGH = np.array([35, 255, 255])
WHITE_LOW   = np.array([0, 0, 140])
WHITE_HIGH  = np.array([180, 100, 255])

AWB_G = 0.89

# ── 전역 ──
latest_jpeg = None
lock = threading.Lock()


def process_loop():
    global latest_jpeg
    cam = Camera()
    cam.start(width=IMG_W, height=IMG_H)
    time.sleep(1)

    M = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)

    while True:
        frame = cam.get_frame()
        # AWB
        r, g, b = cv2.split(frame)
        g = np.clip(g.astype(np.float32) * AWB_G, 0, 255).astype(np.uint8)
        frame = cv2.merge([r, g, b])

        # RAW에서 도로/경계 검출
        hsv_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 노란선
        yellow_raw = cv2.inRange(hsv_raw, YELLOW_LOW, YELLOW_HIGH)

        # 도로 = 어두운 영역 (S<100, V<90)
        road_raw = cv2.inRange(hsv_raw, np.array([0,0,0]), np.array([180,100,90]))
        # 경계 = 도로 아닌 모든 것
        boundary_raw = cv2.bitwise_not(road_raw)
        white_raw = cv2.bitwise_and(boundary_raw, cv2.bitwise_not(yellow_raw))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        white_raw = cv2.morphologyEx(white_raw, cv2.MORPH_OPEN, kernel)

        # BEV 변환
        bev = cv2.warpPerspective(frame, M, (IMG_W, IMG_H))
        yellow = cv2.warpPerspective(yellow_raw, M, (IMG_W, IMG_H))
        white = cv2.warpPerspective(white_raw, M, (IMG_W, IMG_H))
        _, yellow = cv2.threshold(yellow, 127, 255, cv2.THRESH_BINARY)
        _, white = cv2.threshold(white, 127, 255, cv2.THRESH_BINARY)

        # 오버레이
        overlay = bev.copy()
        overlay[yellow > 0] = [0, 255, 255]
        overlay[white > 0]  = [255, 0, 255]

        # RAW에도 검출 결과 표시
        raw_disp = frame.copy()
        raw_disp[yellow_raw > 0] = [0, 255, 255]
        raw_disp[white_raw > 0]  = [255, 0, 255]

        # 원본에 BEV 소스 사각형 표시
        order = np.array([BEV_SRC[0], BEV_SRC[2], BEV_SRC[3], BEV_SRC[1]], dtype=np.int32)
        cv2.polylines(raw_disp, [order], True, (0, 255, 0), 2)

        # 3개 이미지를 가로로 합침 (raw | bev | overlay)
        # 각각 1/2 크기로 줄임
        h2, w2 = IMG_H // 2, IMG_W // 2
        r1 = cv2.resize(raw_disp, (w2, h2))
        r2 = cv2.resize(bev,      (w2, h2))
        r3 = cv2.resize(overlay,  (w2, h2))

        # 라벨 추가
        cv2.putText(r1, 'RAW', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(r2, 'BEV', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(r3, 'LANE', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        combined = np.hstack([r1, r2, r3])

        _, jpeg = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock:
            latest_jpeg = jpeg.tobytes()

        time.sleep(1.0 / FPS)


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'''<html><head><title>Pinky Camera</title>
<style>body{background:#111;margin:0;display:flex;justify-content:center;align-items:center;height:100vh}
img{max-width:100%;height:auto}</style></head>
<body><img src="/stream"></body></html>''')
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
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass  # suppress access logs


if __name__ == '__main__':
    t = threading.Thread(target=process_loop, daemon=True)
    t.start()
    print(f'Camera stream: http://0.0.0.0:{STREAM_PORT}')
    HTTPServer(('0.0.0.0', STREAM_PORT), MJPEGHandler).serve_forever()
