#!/usr/bin/env python3
"""
ARASEO-DALIMI BEV + ROI 캘리브레이션 도구
2단계 캘리브레이션:
  Phase 1: BEV 원근 보정 (캘리브레이션 패드 4꼭짓점 클릭)
  Phase 2: 도로 ROI 다각형 선택 (도로만 포함하도록 경계 클릭)
결과를 lane_params.yaml에 bev_src_pts, bev_dst_pts, roi_pts로 저장
"""
import argparse
import os
import sys
import numpy as np
import cv2

try:
    import yaml
except ImportError:
    print("[오류] PyYAML이 설치되어 있지 않습니다: pip install pyyaml")
    sys.exit(1)

# ── 상수 ──────────────────────────────────────────────────────────
BEV_W, BEV_H = 640, 480
PX_PER_MM = 440.0 / 280.0

# Phase 1 꼭짓점 색상 (TL, TR, BR, BL)
PT_COLORS = [
    (0, 255, 0),    # 녹색
    (255, 255, 0),  # 청록
    (0, 165, 255),  # 주황
    (0, 0, 255),    # 빨강
]
PT_LABELS = ["TL", "TR", "BR", "BL"]

# HSV 임계값
YELLOW_LOW  = np.array([15, 80, 80])
YELLOW_HIGH = np.array([35, 255, 255])
WHITE_LOW   = np.array([0, 0, 180])
WHITE_HIGH  = np.array([180, 60, 255])

# HSV 오버레이 색상
COLOR_YELLOW_OV = (0, 220, 220)   # 청록색
COLOR_WHITE_OV  = (80, 255, 200)  # 연두색


# ── 유틸 ──────────────────────────────────────────────────────────

def compute_dst_pts(pad_w_mm, pad_h_mm):
    """캘리브레이션 패드 크기로 BEV dst 포인트 계산"""
    bev_pad_w = int(pad_w_mm * PX_PER_MM)
    bev_pad_h = int(pad_h_mm * PX_PER_MM)
    x0 = (BEV_W - bev_pad_w) // 2
    y0 = BEV_H - bev_pad_h - 20
    return np.float32([
        [x0,             y0],              # TL
        [x0 + bev_pad_w, y0],              # TR
        [x0 + bev_pad_w, y0 + bev_pad_h],  # BR
        [x0,             y0 + bev_pad_h],  # BL
    ])


def hsv_overlay(bev_img):
    """BEV 이미지에 HSV 차선 마스크 오버레이"""
    hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
    white_mask  = cv2.inRange(hsv, WHITE_LOW, WHITE_HIGH)

    overlay = bev_img.copy()
    overlay[yellow_mask > 0] = COLOR_YELLOW_OV
    overlay[white_mask > 0]  = COLOR_WHITE_OV
    result = cv2.addWeighted(bev_img, 0.5, overlay, 0.5, 0)
    return result, yellow_mask, white_mask


def apply_roi_mask(frame, roi_pts_arr):
    """ROI 다각형 마스크 적용"""
    h, w = frame.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [roi_pts_arr], 255)
    return cv2.bitwise_and(frame, frame, mask=mask)


def draw_dashed_line(img, pt1, pt2, color, thickness=1, gap=10):
    """점선 그리기"""
    dist = np.linalg.norm(np.array(pt2) - np.array(pt1))
    pts_count = int(dist / gap)
    if pts_count == 0:
        return
    for i in range(pts_count):
        if i % 2 == 0:
            t0 = i / pts_count
            t1 = min((i + 1) / pts_count, 1.0)
            p0 = (int(pt1[0] + t0 * (pt2[0] - pt1[0])),
                  int(pt1[1] + t0 * (pt2[1] - pt1[1])))
            p1 = (int(pt1[0] + t1 * (pt2[0] - pt1[0])),
                  int(pt1[1] + t1 * (pt2[1] - pt1[1])))
            cv2.line(img, p0, p1, color, thickness)


def flatten_pts(pts):
    """np.array([[x,y],...]) → [x0,y0,x1,y1,...] 정수 리스트"""
    return [int(v) for v in np.array(pts).flatten()]


# ── yaml 저장/로드 ────────────────────────────────────────────────

def _save_to_yaml(yaml_path, src_pts, dst_pts, roi_pts):
    """기존 yaml 유지하면서 3키만 덮어씀"""
    data = {}
    if os.path.exists(yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f) or {}

    # 구조 보장
    if 'lane_follower' not in data:
        data['lane_follower'] = {}
    if 'ros__parameters' not in data['lane_follower']:
        data['lane_follower']['ros__parameters'] = {}

    params = data['lane_follower']['ros__parameters']
    params['bev_src_pts'] = flatten_pts(src_pts)
    params['bev_dst_pts'] = flatten_pts(dst_pts)
    params['roi_pts']     = flatten_pts(roi_pts)

    with open(yaml_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None, allow_unicode=True,
                  sort_keys=False)

    print(f"\n저장 완료: {yaml_path}")
    print(f"  bev_src_pts: {params['bev_src_pts']}")
    print(f"  bev_dst_pts: {params['bev_dst_pts']}")
    print(f"  roi_pts:     {params['roi_pts']}")


def _load_from_yaml(yaml_path):
    """yaml에서 bev_src_pts, bev_dst_pts, roi_pts 로드"""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    params = data['lane_follower']['ros__parameters']
    src = np.float32(params['bev_src_pts']).reshape(4, 2)
    dst = np.float32(params['bev_dst_pts']).reshape(4, 2)
    roi = None
    if 'roi_pts' in params:
        roi = np.array(params['roi_pts'], dtype=np.int32).reshape(-1, 2)
    else:
        print("[경고] roi_pts가 yaml에 없습니다. ROI 없이 진행합니다.")
    return src, dst, roi


# ── Phase 1: BEV 캘리브레이션 ─────────────────────────────────────

class Phase1:
    def __init__(self, frame, dst_pts):
        self.frame = frame
        self.dst_pts = dst_pts
        self.src_pts = []
        self.show_hsv = False
        self.M = None
        self.win = "[Phase1] BEV 캘리브레이션"
        self.preview_win = "[Phase1] BEV 미리보기"

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.src_pts) < 4:
            self.src_pts.append([x, y])
            print(f"  {PT_LABELS[len(self.src_pts)-1]}: ({x}, {y})")
            if len(self.src_pts) == 4:
                src = np.float32(self.src_pts)
                self.M = cv2.getPerspectiveTransform(src, self.dst_pts)
                print("[Phase1] 4점 수집 완료 — 'n' Phase2 | 'r' 초기화 | 'h' HSV 토글")

    def _draw(self):
        vis = self.frame.copy()
        n = len(self.src_pts)
        for i, pt in enumerate(self.src_pts):
            cv2.circle(vis, tuple(pt), 6, PT_COLORS[i], -1)
            cv2.putText(vis, f"{i+1} {PT_LABELS[i]}", (pt[0]+8, pt[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, PT_COLORS[i], 1)
        if n >= 2:
            overlay = vis.copy()
            if n == 4:
                poly = np.array(self.src_pts, dtype=np.int32)
                cv2.fillPoly(overlay, [poly], (0, 255, 0))
            else:
                for i in range(n - 1):
                    cv2.line(overlay, tuple(self.src_pts[i]),
                             tuple(self.src_pts[i+1]), (0, 255, 0), 2)
            vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)
            # 점 다시 그리기 (오버레이 위에)
            for i, pt in enumerate(self.src_pts):
                cv2.circle(vis, tuple(pt), 6, PT_COLORS[i], -1)

        info = f"클릭: {n}/4"
        if n < 4:
            info += f" | 다음: {PT_LABELS[n]}"
        cv2.putText(vis, info, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return vis

    def _draw_preview(self):
        if self.M is None:
            return None
        bev = cv2.warpPerspective(self.frame, self.M, (BEV_W, BEV_H))
        if self.show_hsv:
            bev, _, _ = hsv_overlay(bev)
        return bev

    def run(self):
        """Phase1 실행. 반환: (src_pts_4x2, M) 또는 None(종료)"""
        print("\n=== Phase 1: BEV 캘리브레이션 ===")
        print("패드 꼭짓점 4개를 TL→TR→BR→BL 순서로 클릭하세요.")
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self._mouse_cb)

        while True:
            vis = self._draw()
            cv2.imshow(self.win, vis)

            preview = self._draw_preview()
            if preview is not None:
                cv2.imshow(self.preview_win, preview)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                return None
            elif key == ord('r'):
                self.src_pts.clear()
                self.M = None
                cv2.destroyWindow(self.preview_win)
                print("[Phase1] 초기화됨")
            elif key == ord('h') and self.M is not None:
                self.show_hsv = not self.show_hsv
                print(f"[Phase1] HSV 오버레이: {'ON' if self.show_hsv else 'OFF'}")
            elif key == ord('n') and self.M is not None:
                cv2.destroyWindow(self.win)
                cv2.destroyWindow(self.preview_win)
                return np.float32(self.src_pts), self.M

        return None


# ── Phase 2: ROI 선택 ────────────────────────────────────────────

class Phase2:
    def __init__(self, frame, src_pts, dst_pts, M):
        self.frame = frame
        self.src_pts = src_pts
        self.dst_pts = dst_pts
        self.M = M
        self.roi_pts = []
        self.confirmed = False
        self.win = "[Phase2] ROI 선택"
        self.result_win = "[Phase2] ROI+BEV 결과"

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_pts.append([x, y])
            print(f"  ROI 점 {len(self.roi_pts)}: ({x}, {y})")

    def _draw(self):
        vis = self.frame.copy()
        n = len(self.roi_pts)
        if n > 0:
            pts_arr = np.array(self.roi_pts, dtype=np.int32)
            # 채워진 반투명 다각형
            if n >= 3:
                overlay = vis.copy()
                cv2.fillPoly(overlay, [pts_arr], (255, 100, 50))
                vis = cv2.addWeighted(vis, 0.6, overlay, 0.4, 0)
            # 실선 연결
            for i in range(n - 1):
                cv2.line(vis, tuple(self.roi_pts[i]),
                         tuple(self.roi_pts[i+1]), (255, 200, 0), 2)
            # 닫힘선: 점선
            if n >= 3:
                draw_dashed_line(vis, tuple(self.roi_pts[-1]),
                                 tuple(self.roi_pts[0]), (255, 200, 0), 1, 10)
            # 점 표시
            for i, pt in enumerate(self.roi_pts):
                cv2.circle(vis, tuple(pt), 5, (255, 200, 0), -1)
                cv2.putText(vis, str(i+1), (pt[0]+7, pt[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        status = "확정됨" if self.confirmed else f"점 {n}개"
        cv2.putText(vis, f"ROI: {status} | Enter:확정 r:초기화 s:저장 b:Phase1",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return vis

    def _draw_result(self):
        if len(self.roi_pts) < 3:
            return None
        roi_arr = np.array(self.roi_pts, dtype=np.int32)
        masked = apply_roi_mask(self.frame, roi_arr)
        bev = cv2.warpPerspective(masked, self.M, (BEV_W, BEV_H))
        result, _, _ = hsv_overlay(bev)
        return result

    def run(self, yaml_path):
        """
        Phase2 실행.
        반환: 'back' (Phase1 복귀), 'quit' (종료), roi_pts (완료)
        """
        print("\n=== Phase 2: ROI 다각형 선택 ===")
        print("도로 경계를 클릭하세요 (3점 이상). Enter로 확정, s로 저장.")
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self._mouse_cb)

        while True:
            vis = self._draw()
            cv2.imshow(self.win, vis)

            result = self._draw_result()
            if result is not None:
                cv2.imshow(self.result_win, result)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                return 'quit'
            elif key == ord('b'):
                cv2.destroyWindow(self.win)
                cv2.destroyWindow(self.result_win)
                return 'back'
            elif key == ord('r'):
                self.roi_pts.clear()
                self.confirmed = False
                cv2.destroyWindow(self.result_win)
                print("[Phase2] ROI 초기화됨")
            elif key == 13:  # Enter
                if len(self.roi_pts) >= 3:
                    self.confirmed = True
                    print(f"[Phase2] ROI 확정: {len(self.roi_pts)}점")
                else:
                    print("[Phase2] 최소 3점을 선택하세요.")
            elif key == ord('s'):
                if len(self.roi_pts) < 3:
                    print("[Phase2] 최소 3점을 선택해야 저장할 수 있습니다.")
                    continue
                roi_arr = np.array(self.roi_pts, dtype=np.int32)
                _save_to_yaml(yaml_path, self.src_pts, self.dst_pts, roi_arr)
                return 'quit'

        return 'quit'


# ── ROS2 토픽 프레임 수신 ─────────────────────────────────────────

def grab_frame_from_topic(topic, timeout=10.0):
    """ROS2 토픽에서 첫 프레임 수신"""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("[오류] rclpy 또는 cv_bridge를 import할 수 없습니다.")
        print("       ROS2 환경을 source 한 뒤 다시 실행하세요.")
        sys.exit(1)

    frame_holder = [None]
    bridge = CvBridge()

    class FrameGrabber(Node):
        def __init__(self):
            super().__init__('bev_calibrator_grabber')
            self.sub = self.create_subscription(
                Image, topic, self._cb, 1)

        def _cb(self, msg):
            frame_holder[0] = bridge.imgmsg_to_cv2(msg, 'bgr8')

    rclpy.init()
    node = FrameGrabber()
    import time
    t0 = time.monotonic()
    while frame_holder[0] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > timeout:
            print(f"[오류] {timeout}초 내에 프레임을 수신하지 못했습니다: {topic}")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    node.destroy_node()
    rclpy.shutdown()
    return frame_holder[0]


def run_topic_mode(topic, dst_pts, yaml_path):
    """ROS2 토픽 실시간 모드"""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("[오류] rclpy 또는 cv_bridge를 import할 수 없습니다.")
        print("       ROS2 환경을 source 한 뒤 다시 실행하세요.")
        sys.exit(1)

    bridge = CvBridge()
    state = {
        'frame': None,
        'frozen_frame': None,
        'frozen': False,
        'phase': 1,
        'src_pts': [],
        'roi_pts': [],
        'M': None,
        'show_hsv': False,
        'roi_confirmed': False,
    }

    rclpy.init()

    class FrameGrabber(rclpy.node.Node):
        def __init__(self):
            super().__init__('bev_calibrator_live')
            self.sub = self.create_subscription(
                Image, topic, self._cb, 1)

        def _cb(self, msg):
            state['frame'] = bridge.imgmsg_to_cv2(msg, 'bgr8')

    node = FrameGrabber()

    # 첫 프레임 대기
    import time
    t0 = time.monotonic()
    while state['frame'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > 10.0:
            print(f"[오류] 10초 내에 프레임을 수신하지 못했습니다: {topic}")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    print(f"[실시간] 프레임 수신 시작 — 'f'로 고정/해제")

    def mouse_cb(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if state['phase'] == 1 and len(state['src_pts']) < 4:
            state['src_pts'].append([x, y])
            print(f"  {PT_LABELS[len(state['src_pts'])-1]}: ({x}, {y})")
            if len(state['src_pts']) == 4:
                src = np.float32(state['src_pts'])
                state['M'] = cv2.getPerspectiveTransform(src, dst_pts)
                print("[Phase1] 4점 수집 완료 — 'n' Phase2 전환")
        elif state['phase'] == 2:
            state['roi_pts'].append([x, y])
            print(f"  ROI 점 {len(state['roi_pts'])}: ({x}, {y})")

    win_name = "[Phase1] BEV 캘리브레이션"
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win_name, mouse_cb)

    while True:
        rclpy.spin_once(node, timeout_sec=0.01)

        if state['frame'] is None:
            continue

        if not state['frozen']:
            current = state['frame'].copy()
            state['frozen_frame'] = current
        else:
            current = state['frozen_frame']

        # 타이틀 업데이트
        freeze_tag = "[FROZEN]" if state['frozen'] else "[LIVE]"
        phase_tag = f"Phase{state['phase']}"

        vis = current.copy()

        if state['phase'] == 1:
            title = f"[{phase_tag}] {freeze_tag} BEV 캘리브레이션"
            n = len(state['src_pts'])
            for i, pt in enumerate(state['src_pts']):
                cv2.circle(vis, tuple(pt), 6, PT_COLORS[i], -1)
                cv2.putText(vis, f"{i+1} {PT_LABELS[i]}",
                            (pt[0]+8, pt[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, PT_COLORS[i], 1)
            if n >= 2:
                overlay = vis.copy()
                if n == 4:
                    poly = np.array(state['src_pts'], dtype=np.int32)
                    cv2.fillPoly(overlay, [poly], (0, 255, 0))
                else:
                    for i in range(n - 1):
                        cv2.line(overlay, tuple(state['src_pts'][i]),
                                 tuple(state['src_pts'][i+1]), (0, 255, 0), 2)
                vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)
                for i, pt in enumerate(state['src_pts']):
                    cv2.circle(vis, tuple(pt), 6, PT_COLORS[i], -1)

            info = f"{freeze_tag} 클릭: {n}/4"
            if n < 4:
                info += f" | 다음: {PT_LABELS[n]}"
            cv2.putText(vis, info, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if state['M'] is not None:
                bev = cv2.warpPerspective(current, state['M'], (BEV_W, BEV_H))
                if state['show_hsv']:
                    bev, _, _ = hsv_overlay(bev)
                cv2.imshow("[Phase1] BEV 미리보기", bev)

        else:  # Phase 2
            title = f"[{phase_tag}] {freeze_tag} ROI 선택"
            n = len(state['roi_pts'])
            if n > 0:
                pts_arr = np.array(state['roi_pts'], dtype=np.int32)
                if n >= 3:
                    overlay = vis.copy()
                    cv2.fillPoly(overlay, [pts_arr], (255, 100, 50))
                    vis = cv2.addWeighted(vis, 0.6, overlay, 0.4, 0)
                for i in range(n - 1):
                    cv2.line(vis, tuple(state['roi_pts'][i]),
                             tuple(state['roi_pts'][i+1]), (255, 200, 0), 2)
                if n >= 3:
                    draw_dashed_line(vis, tuple(state['roi_pts'][-1]),
                                     tuple(state['roi_pts'][0]),
                                     (255, 200, 0), 1, 10)
                for i, pt in enumerate(state['roi_pts']):
                    cv2.circle(vis, tuple(pt), 5, (255, 200, 0), -1)
                    cv2.putText(vis, str(i+1), (pt[0]+7, pt[1]-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (255, 255, 255), 1)

            status = "확정됨" if state['roi_confirmed'] else f"점 {n}개"
            cv2.putText(vis, f"{freeze_tag} ROI: {status}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)

            if n >= 3 and state['M'] is not None:
                roi_arr = np.array(state['roi_pts'], dtype=np.int32)
                masked = apply_roi_mask(current, roi_arr)
                bev = cv2.warpPerspective(masked, state['M'], (BEV_W, BEV_H))
                result, _, _ = hsv_overlay(bev)
                cv2.imshow("[Phase2] ROI+BEV 결과", result)

        cv2.setWindowTitle(win_name, title)
        cv2.imshow(win_name, vis)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            state['frozen'] = not state['frozen']
            tag = "고정" if state['frozen'] else "실시간"
            print(f"[프레임] {tag}")
        elif key == ord('r'):
            if state['phase'] == 1:
                state['src_pts'].clear()
                state['M'] = None
                cv2.destroyWindow("[Phase1] BEV 미리보기")
                print("[Phase1] 초기화됨")
            else:
                state['roi_pts'].clear()
                state['roi_confirmed'] = False
                cv2.destroyWindow("[Phase2] ROI+BEV 결과")
                print("[Phase2] ROI 초기화됨")
        elif key == ord('h') and state['phase'] == 1 and state['M'] is not None:
            state['show_hsv'] = not state['show_hsv']
            print(f"[Phase1] HSV 오버레이: {'ON' if state['show_hsv'] else 'OFF'}")
        elif key == ord('n') and state['phase'] == 1 and state['M'] is not None:
            state['phase'] = 2
            cv2.destroyWindow("[Phase1] BEV 미리보기")
            print("[Phase2] ROI 다각형 선택으로 전환")
        elif key == ord('b') and state['phase'] == 2:
            state['phase'] = 1
            state['roi_pts'].clear()
            state['roi_confirmed'] = False
            cv2.destroyWindow("[Phase2] ROI+BEV 결과")
            print("[Phase1] BEV 캘리브레이션으로 복귀")
        elif key == 13 and state['phase'] == 2:  # Enter
            if len(state['roi_pts']) >= 3:
                state['roi_confirmed'] = True
                print(f"[Phase2] ROI 확정: {len(state['roi_pts'])}점")
            else:
                print("[Phase2] 최소 3점을 선택하세요.")
        elif key == ord('s') and state['phase'] == 2:
            if len(state['roi_pts']) < 3:
                print("[Phase2] 최소 3점을 선택해야 저장할 수 있습니다.")
                continue
            roi_arr = np.array(state['roi_pts'], dtype=np.int32)
            _save_to_yaml(yaml_path,
                          np.float32(state['src_pts']), dst_pts, roi_arr)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


# ── verify 모드 ───────────────────────────────────────────────────

def run_verify(image_path, yaml_path):
    """저장된 yaml 결과를 3개 창으로 검증"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"[오류] 이미지를 로드할 수 없습니다: {image_path}")
        sys.exit(1)

    src, dst, roi = _load_from_yaml(yaml_path)
    M = cv2.getPerspectiveTransform(src, dst)

    # 창 1: 원본 + ROI 경계선
    vis_original = frame.copy()
    if roi is not None:
        cv2.polylines(vis_original, [roi], isClosed=True,
                      color=(0, 255, 255), thickness=2)
    cv2.imshow("[Verify] 원본 + ROI", vis_original)

    # 창 2: ROI 마스크 → BEV
    if roi is not None:
        masked = apply_roi_mask(frame, roi)
    else:
        masked = frame
    bev = cv2.warpPerspective(masked, M, (BEV_W, BEV_H))
    cv2.imshow("[Verify] ROI+BEV", bev)

    # 창 3: BEV + HSV 오버레이
    bev_hsv, yellow_mask, white_mask = hsv_overlay(bev)
    cv2.imshow("[Verify] BEV+HSV 오버레이", bev_hsv)

    # 차선 픽셀 수 출력
    yellow_px = int(np.sum(yellow_mask) // 255)
    white_px  = int(np.sum(white_mask) // 255)
    total_px  = yellow_px + white_px
    print(f"\n=== 검증 결과 ===")
    print(f"  황색 차선 픽셀: {yellow_px}")
    print(f"  백색 차선 픽셀: {white_px}")
    print(f"  합계:           {total_px}")
    if total_px < 500:
        print("  [경고] 차선 픽셀이 500 미만입니다. 재캘리브레이션을 권고합니다.")
    else:
        print("  [OK] 차선 검출 양호")

    print("\n아무 키나 누르면 종료합니다.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ── 이미지 모드 ───────────────────────────────────────────────────

def run_image_mode(image_path, dst_pts, yaml_path):
    """정지 이미지로 2단계 캘리브레이션"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"[오류] 이미지를 로드할 수 없습니다: {image_path}")
        sys.exit(1)

    while True:
        # Phase 1
        p1 = Phase1(frame, dst_pts)
        result = p1.run()
        if result is None:
            break
        src_pts, M = result

        # Phase 2
        while True:
            p2 = Phase2(frame, src_pts, dst_pts, M)
            action = p2.run(yaml_path)
            if action == 'quit':
                cv2.destroyAllWindows()
                return
            elif action == 'back':
                break  # Phase 1로 복귀

    cv2.destroyAllWindows()


# ── main ──────────────────────────────────────────────────────────

def main():
    default_yaml = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'lane_params.yaml')

    parser = argparse.ArgumentParser(
        description='ARASEO-DALIMI BEV + ROI 캘리브레이션 도구')
    parser.add_argument('--image', type=str, default=None,
                        help='정지 이미지 파일로 캘리브레이션')
    parser.add_argument('--topic', type=str, default=None,
                        help='ROS2 카메라 토픽으로 실시간 캘리브레이션')
    parser.add_argument('--verify', type=str, default=None,
                        help='저장된 yaml로 결과 검증 (이미지 경로)')
    parser.add_argument('--yaml', type=str, default=default_yaml,
                        help=f'저장할 yaml 경로 (기본: {default_yaml})')
    parser.add_argument('--robot-id', type=int, default=0,
                        help='로봇 ID (기본: 0)')
    parser.add_argument('--pad-w', type=int, default=200,
                        help='캘리브레이션 패드 가로 폭 mm (기본: 200)')
    parser.add_argument('--pad-h', type=int, default=150,
                        help='캘리브레이션 패드 세로 높이 mm (기본: 150)')
    parser.add_argument('--mode', type=str, default=None,
                        choices=['image', 'topic', 'verify'],
                        help='실행 모드 (launch에서 주입 시 사용)')
    args = parser.parse_args()

    dst_pts = compute_dst_pts(args.pad_w, args.pad_h)

    print("ARASEO-DALIMI BEV+ROI 캘리브레이터")
    print(f"  패드 크기: {args.pad_w}×{args.pad_h} mm")
    print(f"  yaml 경로: {args.yaml}")

    # --mode 가 있으면 해당 모드 강제 적용
    if args.mode == 'verify':
        args.verify = args.verify or args.image
    elif args.mode == 'topic':
        args.topic = args.topic or f'/robot_{args.robot_id}/camera/image_raw'
    elif args.mode == 'image':
        pass   # args.image 그대로 사용

    if args.verify:
        run_verify(args.verify, args.yaml)
    elif args.topic:
        run_topic_mode(args.topic, dst_pts, args.yaml)
    elif args.image:
        run_image_mode(args.image, dst_pts, args.yaml)
    else:
        parser.print_help()
        print("\n--image, --topic, --verify, --mode 중 하나를 지정하세요.")


if __name__ == '__main__':
    main()
