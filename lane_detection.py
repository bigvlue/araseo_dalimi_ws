#!/usr/bin/env python3
"""
ARASEO-DALIMI Lane Detection (LAB + CLAHE + Adaptive Threshold)
- 실내 형광등 / 부분 그림자 환경에 강건한 차선 검출
- Flask MJPEG 웹 모니터링 + 실시간 파라미터 슬라이더
실행: python3 lane_detection.py
접속: http://<Raspberry Pi IP>:5000
"""
import cv2
import numpy as np
import threading
import time
from flask import Flask, Response, request, jsonify, render_template_string
from picamera2 import Picamera2

# ═══════════════════════════════════════════════════════════════════
#  튜닝 파라미터 (상단 상수)
# ═══════════════════════════════════════════════════════════════════

# CLAHE 전처리
CLAHE_CLIP_LIMIT = 3.0
CLAHE_TILE_SIZE = 8       # tileGridSize = (N, N)

# LAB 황색 중앙선 검출 (B 채널: 높을수록 황색)
YELLOW_B_THRESH_BLOCK = 31    # adaptiveThreshold blockSize (홀수)
YELLOW_B_THRESH_C = -8        # adaptiveThreshold C (음수 → 밝은 쪽 선택)
YELLOW_B_MIN = 135            # B채널 최소값 (황색 필터 전처리)

# LAB 백색 차선 검출 (L 채널: 높을수록 밝음)
WHITE_L_THRESH_BLOCK = 31
WHITE_L_THRESH_C = -12
WHITE_L_MIN = 170             # L채널 최소값 (백색 필터 전처리)

# 모폴로지
MORPH_KERNEL_SIZE = 3
MORPH_OPEN_ITER = 1
MORPH_CLOSE_ITER = 1

# BEV 변환 (640x480 기준)
BEV_SRC = np.float32([[11, 440], [628, 440], [209, 280], [422, 280]])
BEV_DST = np.float32([[80, 480], [560, 480], [80, 0], [560, 0]])

# ROI (도로 영역만 사용)
ROI_PTS = np.array([[0, 480], [640, 480], [640, 200], [0, 200]], dtype=np.int32)

# 슬라이딩 윈도우
N_WINDOWS = 9
MARGIN_PX = 60
MIN_PIXELS = 30

# 카메라
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 웹 서버
WEB_PORT = 5000
JPEG_QUALITY = 70

# ═══════════════════════════════════════════════════════════════════
#  전역 상태 (스레드 간 공유)
# ═══════════════════════════════════════════════════════════════════

_lock = threading.Lock()
_state = {
    'frame_raw': None,
    'mask_yellow': None,
    'mask_white': None,
    'mask_combined': None,
    'frame_overlay': None,
    'offset': 0.0,
    'left_detected': False,
    'right_detected': False,
}

# 동적 파라미터 (웹 슬라이더로 변경 가능)
_params = {
    'clahe_clip': CLAHE_CLIP_LIMIT,
    'clahe_tile': CLAHE_TILE_SIZE,
    'yellow_b_block': YELLOW_B_THRESH_BLOCK,
    'yellow_b_c': YELLOW_B_THRESH_C,
    'yellow_b_min': YELLOW_B_MIN,
    'white_l_block': WHITE_L_THRESH_BLOCK,
    'white_l_c': WHITE_L_THRESH_C,
    'white_l_min': WHITE_L_MIN,
}
_params_lock = threading.Lock()


def get_params():
    with _params_lock:
        return dict(_params)


def set_param(key, value):
    with _params_lock:
        if key in _params:
            _params[key] = value


# ═══════════════════════════════════════════════════════════════════
#  1. 전처리
# ═══════════════════════════════════════════════════════════════════

def preprocess(frame, clahe_clip, clahe_tile):
    """ROI 적용 + CLAHE (LAB L채널) → LAB 이미지 반환"""
    # ROI 마스크
    roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillPoly(roi_mask, [ROI_PTS], 255)
    masked = cv2.bitwise_and(frame, frame, mask=roi_mask)

    # LAB 변환
    lab = cv2.cvtColor(masked, cv2.COLOR_BGR2LAB)

    # CLAHE를 L채널에 적용 → 조명 균일화
    tile = max(1, int(clahe_tile))
    clahe = cv2.createCLAHE(clipLimit=float(clahe_clip),
                            tileGridSize=(tile, tile))
    l_ch, a_ch, b_ch = cv2.split(lab)
    l_ch = clahe.apply(l_ch)
    lab = cv2.merge([l_ch, a_ch, b_ch])

    return lab, roi_mask


# ═══════════════════════════════════════════════════════════════════
#  2. 차선 검출
# ═══════════════════════════════════════════════════════════════════

def detect_yellow(lab, params):
    """LAB B채널 기반 황색 중앙선 검출"""
    b_ch = lab[:, :, 2]  # B채널: 높을수록 yellow

    # B채널 최소값 필터 (확실한 황색 영역만)
    b_min = int(params['yellow_b_min'])
    _, b_thresh = cv2.threshold(b_ch, b_min, 255, cv2.THRESH_BINARY)

    # Adaptive Threshold (그림자에도 강건)
    block = int(params['yellow_b_block'])
    if block % 2 == 0:
        block += 1
    block = max(3, block)
    c_val = int(params['yellow_b_c'])

    b_adaptive = cv2.adaptiveThreshold(
        b_ch, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, block, c_val)

    # 두 마스크 교집합 (전역 필터 + 적응형)
    mask = cv2.bitwise_and(b_thresh, b_adaptive)

    # 모폴로지
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                       (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                            iterations=MORPH_OPEN_ITER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                            iterations=MORPH_CLOSE_ITER)
    return mask


def detect_white(lab, params):
    """LAB L채널 기반 백색 차선 검출"""
    l_ch = lab[:, :, 0]  # L채널: 높을수록 밝음

    # L채널 최소값 필터 (확실한 백색 영역만)
    l_min = int(params['white_l_min'])
    _, l_thresh = cv2.threshold(l_ch, l_min, 255, cv2.THRESH_BINARY)

    # Adaptive Threshold
    block = int(params['white_l_block'])
    if block % 2 == 0:
        block += 1
    block = max(3, block)
    c_val = int(params['white_l_c'])

    l_adaptive = cv2.adaptiveThreshold(
        l_ch, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, block, c_val)

    # 두 마스크 교집합
    mask = cv2.bitwise_and(l_thresh, l_adaptive)

    # 황색 영역 제거 (B채널 높은 부분 제외)
    b_ch = lab[:, :, 2]
    yellow_region = cv2.threshold(b_ch, int(params['yellow_b_min']),
                                  255, cv2.THRESH_BINARY)[1]
    mask = cv2.bitwise_and(mask, cv2.bitwise_not(yellow_region))

    # 모폴로지
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                       (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                            iterations=MORPH_OPEN_ITER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                            iterations=MORPH_CLOSE_ITER)
    return mask


# ═══════════════════════════════════════════════════════════════════
#  3. 중앙 offset 계산
# ═══════════════════════════════════════════════════════════════════

def compute_offset(yellow_bev, white_bev):
    """
    BEV 마스크에서 슬라이딩 윈도우로 좌/우 차선 중앙점 → offset 계산.
    반환: (offset, left_fit, right_fit, left_det, right_det)
      offset: -1.0(좌치우침) ~ +1.0(우치우침), 0.0=중앙
    """
    h, w = yellow_bev.shape

    # 히스토그램 기저점
    yellow_hist = np.sum(yellow_bev[h // 2:, :], axis=0)
    white_hist = np.sum(white_bev[h // 2:, :], axis=0)
    mid = w // 2

    left_base = int(np.argmax(yellow_hist[:mid])) if yellow_hist[:mid].max() > 0 else mid // 2
    right_base = int(np.argmax(white_hist[mid:]) + mid) if white_hist[mid:].max() > 0 else mid + (w - mid) // 2

    win_h = h // N_WINDOWS
    left_x, left_y = [], []
    right_x, right_y = [], []
    lx, rx = left_base, right_base

    y_nz = yellow_bev.nonzero()
    ly_arr, lx_arr = y_nz[0], y_nz[1]
    w_nz = white_bev.nonzero()
    wy_arr, wx_arr = w_nz[0], w_nz[1]

    for win in range(N_WINDOWS):
        y_low = h - (win + 1) * win_h
        y_high = h - win * win_h

        good_l = ((ly_arr >= y_low) & (ly_arr < y_high) &
                  (lx_arr >= lx - MARGIN_PX) & (lx_arr < lx + MARGIN_PX))
        good_r = ((wy_arr >= y_low) & (wy_arr < y_high) &
                  (wx_arr >= rx - MARGIN_PX) & (wx_arr < rx + MARGIN_PX))

        gl_idx = good_l.nonzero()[0]
        gr_idx = good_r.nonzero()[0]

        left_x.extend(lx_arr[gl_idx])
        left_y.extend(ly_arr[gl_idx])
        right_x.extend(wx_arr[gr_idx])
        right_y.extend(wy_arr[gr_idx])

        if len(gl_idx) > MIN_PIXELS:
            lx = int(np.mean(lx_arr[gl_idx]))
        if len(gr_idx) > MIN_PIXELS:
            rx = int(np.mean(wx_arr[gr_idx]))

    left_det = len(left_x) > MIN_PIXELS * 2
    right_det = len(right_x) > MIN_PIXELS * 2

    left_fit = np.polyfit(left_y, left_x, 2) if left_det else None
    right_fit = np.polyfit(right_y, right_x, 2) if right_det else None

    # offset 계산
    y_eval = float(h - 1)
    center_x = w / 2.0

    if left_fit is not None and right_fit is not None:
        lx_val = np.polyval(left_fit, y_eval)
        rx_val = np.polyval(right_fit, y_eval)
        road_center = (lx_val + rx_val) / 2.0
    elif left_fit is not None:
        road_center = np.polyval(left_fit, y_eval) + w * 0.25
    elif right_fit is not None:
        road_center = np.polyval(right_fit, y_eval) - w * 0.25
    else:
        road_center = center_x

    offset = float(np.clip((road_center - center_x) / (w / 2.0), -1.0, 1.0))
    return offset, left_fit, right_fit, left_det, right_det


# ═══════════════════════════════════════════════════════════════════
#  4. 시각화
# ═══════════════════════════════════════════════════════════════════

def draw_overlay(frame, yellow_bev, white_bev, left_fit, right_fit,
                 left_det, right_det, offset):
    """원본 프레임 위에 BEV 역변환 차선 오버레이"""
    h, w = frame.shape[:2]
    M_bev = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)
    M_inv = cv2.getPerspectiveTransform(BEV_DST, BEV_SRC)

    overlay = frame.copy()
    plot_y = np.linspace(0, h - 1, h)

    # 차선 다항식 곡선을 BEV 위에 그린 뒤 역변환
    bev_canvas = np.zeros((h, w, 3), dtype=np.uint8)

    if left_fit is not None:
        pts_x = np.polyval(left_fit, plot_y).astype(np.int32)
        pts = np.column_stack([pts_x, plot_y.astype(np.int32)])
        pts = pts[(pts[:, 0] >= 0) & (pts[:, 0] < w)]
        if len(pts) > 1:
            color = (0, 255, 255) if left_det else (0, 128, 128)
            cv2.polylines(bev_canvas, [pts], False, color, 4)

    if right_fit is not None:
        pts_x = np.polyval(right_fit, plot_y).astype(np.int32)
        pts = np.column_stack([pts_x, plot_y.astype(np.int32)])
        pts = pts[(pts[:, 0] >= 0) & (pts[:, 0] < w)]
        if len(pts) > 1:
            color = (255, 255, 255) if right_det else (128, 128, 128)
            cv2.polylines(bev_canvas, [pts], False, color, 4)

    # 차로 영역 채우기
    if left_fit is not None and right_fit is not None:
        left_pts_x = np.polyval(left_fit, plot_y).astype(np.int32)
        right_pts_x = np.polyval(right_fit, plot_y).astype(np.int32)
        left_pts = np.column_stack([left_pts_x, plot_y.astype(np.int32)])
        right_pts = np.column_stack([right_pts_x, plot_y.astype(np.int32)])
        fill_pts = np.vstack([left_pts, right_pts[::-1]])
        fill_pts[:, 0] = np.clip(fill_pts[:, 0], 0, w - 1)
        cv2.fillPoly(bev_canvas, [fill_pts], (0, 80, 0))

    # BEV → 원본 시점으로 역변환
    warped_back = cv2.warpPerspective(bev_canvas, M_inv, (w, h))
    mask_nonzero = warped_back.sum(axis=2) > 0
    overlay[mask_nonzero] = cv2.addWeighted(
        overlay, 0.6, warped_back, 0.4, 0)[mask_nonzero]

    # offset 텍스트
    direction = "LEFT" if offset < -0.05 else ("RIGHT" if offset > 0.05 else "CENTER")
    cv2.putText(overlay, f"offset: {offset:+.3f} ({direction})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(overlay, f"L:{'OK' if left_det else '--'}  R:{'OK' if right_det else '--'}",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # 중심선 가이드
    cx = int(w / 2 + offset * w / 2)
    cv2.line(overlay, (w // 2, h), (w // 2, h - 40), (0, 0, 255), 2)
    cv2.line(overlay, (cx, h), (cx, h - 40), (0, 255, 0), 3)

    return overlay


# ═══════════════════════════════════════════════════════════════════
#  메인 검출 루프
# ═══════════════════════════════════════════════════════════════════

def detection_loop():
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"format": "BGR888", "size": (FRAME_WIDTH, FRAME_HEIGHT)})
    picam.configure(config)
    picam.start()
    time.sleep(0.5)  # 센서 안정화

    M_bev = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)
    print("[INFO] 차선 검출 시작 (Picamera2)")

    while True:
        frame = picam.capture_array()
        if frame is None:
            time.sleep(0.01)
            continue

        params = get_params()

        # 1. 전처리
        lab, roi_mask = preprocess(frame, params['clahe_clip'],
                                   params['clahe_tile'])

        # 2. 검출
        yellow_mask = detect_yellow(lab, params)
        white_mask = detect_white(lab, params)
        yellow_mask = cv2.bitwise_and(yellow_mask, roi_mask)
        white_mask = cv2.bitwise_and(white_mask, roi_mask)
        combined = cv2.bitwise_or(yellow_mask, white_mask)

        # 3. BEV 변환
        yellow_bev = cv2.warpPerspective(yellow_mask, M_bev,
                                         (FRAME_WIDTH, FRAME_HEIGHT))
        white_bev = cv2.warpPerspective(white_mask, M_bev,
                                        (FRAME_WIDTH, FRAME_HEIGHT))
        _, yellow_bev = cv2.threshold(yellow_bev, 127, 255, cv2.THRESH_BINARY)
        _, white_bev = cv2.threshold(white_bev, 127, 255, cv2.THRESH_BINARY)

        # 4. 중앙 offset
        offset, left_fit, right_fit, left_det, right_det = compute_offset(
            yellow_bev, white_bev)

        # 5. 시각화
        overlay = draw_overlay(frame, yellow_bev, white_bev,
                               left_fit, right_fit,
                               left_det, right_det, offset)

        # 상태 업데이트
        with _lock:
            _state['frame_raw'] = frame
            _state['mask_yellow'] = yellow_mask
            _state['mask_white'] = white_mask
            _state['mask_combined'] = combined
            _state['frame_overlay'] = overlay
            _state['offset'] = offset
            _state['left_detected'] = left_det
            _state['right_detected'] = right_det

    picam.stop()


# ═══════════════════════════════════════════════════════════════════
#  Flask 웹 서버
# ═══════════════════════════════════════════════════════════════════

app = Flask(__name__)

HTML_PAGE = """
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>ARASEO Lane Detection</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #1a1a2e; color: #eee; font-family: 'Segoe UI', sans-serif; }
  h1 { text-align: center; padding: 12px 0 4px; font-size: 1.3em; color: #0ff; }
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 6px;
    padding: 6px 10px;
    max-width: 1200px;
    margin: 0 auto;
  }
  .cell { text-align: center; }
  .cell img { width: 100%; border-radius: 4px; border: 1px solid #333; }
  .cell .label { font-size: 0.85em; color: #aaa; margin-top: 2px; }
  .offset-box {
    text-align: center; padding: 10px; font-size: 1.6em; font-weight: bold;
  }
  .offset-box span { color: #0f0; }
  .controls {
    max-width: 600px; margin: 10px auto; padding: 10px 20px;
    background: #16213e; border-radius: 8px;
  }
  .controls h2 { font-size: 1em; color: #0ff; margin-bottom: 8px; }
  .slider-row {
    display: flex; align-items: center; margin: 5px 0; font-size: 0.85em;
  }
  .slider-row label { width: 150px; text-align: right; padding-right: 10px; }
  .slider-row input[type=range] { flex: 1; }
  .slider-row .val { width: 50px; text-align: center; color: #0ff; }
</style>
</head><body>

<h1>ARASEO-DALIMI Lane Detection Monitor</h1>

<div class="grid">
  <div class="cell">
    <img src="/stream/raw" alt="raw"><div class="label">Original</div>
  </div>
  <div class="cell">
    <img src="/stream/yellow" alt="yellow"><div class="label">Yellow Mask (B ch)</div>
  </div>
  <div class="cell">
    <img src="/stream/white" alt="white"><div class="label">White Mask (L ch)</div>
  </div>
  <div class="cell">
    <img src="/stream/combined" alt="combined"><div class="label">Combined Mask</div>
  </div>
  <div class="cell">
    <img src="/stream/overlay" alt="overlay"><div class="label">Overlay</div>
  </div>
  <div class="cell offset-box">
    <div>Offset</div>
    <div><span id="offset-val">0.000</span></div>
    <div style="font-size:0.5em; color:#aaa;">
      L: <span id="det-l">--</span> &nbsp; R: <span id="det-r">--</span>
    </div>
  </div>
</div>

<div class="controls">
  <h2>Parameters</h2>
  <div class="slider-row">
    <label>CLAHE clipLimit</label>
    <input type="range" min="1" max="10" step="0.5" value="3" id="s_clahe_clip">
    <div class="val" id="v_clahe_clip">3.0</div>
  </div>
  <div class="slider-row">
    <label>CLAHE tileSize</label>
    <input type="range" min="2" max="16" step="1" value="8" id="s_clahe_tile">
    <div class="val" id="v_clahe_tile">8</div>
  </div>
  <div class="slider-row">
    <label>Yellow B block</label>
    <input type="range" min="3" max="61" step="2" value="31" id="s_yellow_b_block">
    <div class="val" id="v_yellow_b_block">31</div>
  </div>
  <div class="slider-row">
    <label>Yellow B C</label>
    <input type="range" min="-30" max="0" step="1" value="-8" id="s_yellow_b_c">
    <div class="val" id="v_yellow_b_c">-8</div>
  </div>
  <div class="slider-row">
    <label>Yellow B min</label>
    <input type="range" min="100" max="200" step="1" value="135" id="s_yellow_b_min">
    <div class="val" id="v_yellow_b_min">135</div>
  </div>
  <div class="slider-row">
    <label>White L block</label>
    <input type="range" min="3" max="61" step="2" value="31" id="s_white_l_block">
    <div class="val" id="v_white_l_block">31</div>
  </div>
  <div class="slider-row">
    <label>White L C</label>
    <input type="range" min="-30" max="0" step="1" value="-12" id="s_white_l_c">
    <div class="val" id="v_white_l_c">-12</div>
  </div>
  <div class="slider-row">
    <label>White L min</label>
    <input type="range" min="100" max="250" step="1" value="170" id="s_white_l_min">
    <div class="val" id="v_white_l_min">170</div>
  </div>
</div>

<script>
const params = [
  'clahe_clip', 'clahe_tile',
  'yellow_b_block', 'yellow_b_c', 'yellow_b_min',
  'white_l_block', 'white_l_c', 'white_l_min'
];
params.forEach(p => {
  const slider = document.getElementById('s_' + p);
  const val = document.getElementById('v_' + p);
  slider.addEventListener('input', () => {
    val.textContent = slider.value;
    fetch('/param', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({key: p, value: parseFloat(slider.value)})
    });
  });
});

// offset 폴링
setInterval(() => {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('offset-val').textContent = d.offset.toFixed(3);
    document.getElementById('det-l').textContent = d.left ? 'OK' : '--';
    document.getElementById('det-r').textContent = d.right ? 'OK' : '--';
    document.getElementById('det-l').style.color = d.left ? '#0f0' : '#f00';
    document.getElementById('det-r').style.color = d.right ? '#0f0' : '#f00';
  });
}, 200);
</script>
</body></html>
"""


@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


@app.route('/status')
def status():
    with _lock:
        return jsonify({
            'offset': _state['offset'],
            'left': _state['left_detected'],
            'right': _state['right_detected'],
        })


@app.route('/param', methods=['POST'])
def update_param():
    data = request.get_json(force=True)
    key = data.get('key', '')
    value = data.get('value', 0)
    set_param(key, value)
    return jsonify({'ok': True, 'key': key, 'value': value})


def _mjpeg_stream(key):
    """MJPEG 스트림 제너레이터"""
    while True:
        with _lock:
            img = _state.get(key)
        if img is None:
            time.sleep(0.05)
            continue

        # 마스크(1ch)는 그레이→JPEG, 컬러(3ch)는 그대로
        if len(img.shape) == 2:
            encode_img = img
        else:
            encode_img = img

        _, buf = cv2.imencode('.jpg', encode_img,
                              [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               buf.tobytes() + b'\r\n')
        time.sleep(0.05)


@app.route('/stream/<name>')
def stream(name):
    key_map = {
        'raw': 'frame_raw',
        'yellow': 'mask_yellow',
        'white': 'mask_white',
        'combined': 'mask_combined',
        'overlay': 'frame_overlay',
    }
    key = key_map.get(name)
    if key is None:
        return "Not found", 404
    return Response(_mjpeg_stream(key),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# ═══════════════════════════════════════════════════════════════════
#  엔트리포인트
# ═══════════════════════════════════════════════════════════════════

def main():
    # 검출 스레드
    det_thread = threading.Thread(target=detection_loop, daemon=True)
    det_thread.start()
    print(f"[INFO] 웹 모니터링: http://0.0.0.0:{WEB_PORT}")

    # Flask (메인 스레드)
    app.run(host='0.0.0.0', port=WEB_PORT, threaded=True)


if __name__ == '__main__':
    main()
