#!/usr/bin/env python3
"""
ARASEO-DALIMI Lane Follower Node
BEV 변환 + HSV 차선 검출 + Pure Pursuit 제어기
한국 도로교통법: 황색 중앙선 좌측 유지, 백색 정지선 앞 정지
"""
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from araseo_dalimi_interfaces.msg import LaneInfo, TrafficSign


class LaneFollowerNode(Node):

    def __init__(self):
        super().__init__('lane_follower')

        # 파라미터 로드
        self.declare_parameters('', [
            ('robot_id',             0),
            ('image_width',          640),
            ('image_height',         480),
            ('bev_src_pts',          [11,440,628,440,209,280,422,280]),
            ('bev_dst_pts',          [80,480,560,480,80,0,560,0]),
            ('yellow_h_low',         10),
            ('yellow_h_high',        40),
            ('yellow_s_low',         40),
            ('yellow_s_high',        255),
            ('yellow_v_low',         40),
            ('yellow_v_high',        255),
            ('white_s_high',         100),
            ('white_v_low',          140),
            ('n_windows',            9),
            ('margin_px',            60),
            ('min_pixels',           30),
            ('lookahead_straight_m', 0.30),
            ('lookahead_curve_m',    0.15),
            ('base_speed_mps',       0.15),
            ('max_steering_rad',     0.524),
            ('wheelbase_m',          0.110),
            ('stop_line_roi_y',      420),
            ('stop_line_threshold',  80),
            ('publish_hz',           20.0),
            ('roi_pts', [0, 480, 640, 480, 640, 200, 0, 200]),
            ('ir_stop_min_sensors',  2),
            ('ir_lane_assist',       True),
            ('ir_lane_error_gain',   0.4),
            ('intersection_yellow_min_px',   300),
            ('intersection_aspect_min',      0.3),
        ])

        self.robot_id = self.get_parameter('robot_id').value
        self._bridge  = CvBridge()
        self._load_params()

        # 구독
        self.create_subscription(
            Image,
            f'/robot_{self.robot_id}/camera/image_raw',
            self._image_callback,
            1  # QoS depth=1, 최신 프레임만 처리
        )

        # 발행
        self._cmd_pub  = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10)
        self._lane_pub = self.create_publisher(
            LaneInfo, f'/robot_{self.robot_id}/lane_info', 10)
        self._sign_pub = self.create_publisher(
            TrafficSign, f'/robot_{self.robot_id}/traffic_sign', 10)

        self.create_subscription(
            Int16MultiArray,
            f'/robot_{self.robot_id}/ir_sensors',
            self._ir_callback,
            10)

        # 이전 프레임 차선 상태 (슬라이딩 윈도우 warm-start)
        self._prev_left_fit  = None
        self._prev_right_fit = None
        self._stop_flag      = False
        self._stop_suppressed_until = 0.0   # resume 후 재래치 억제 만료 시각

        from std_msgs.msg import Bool
        self.create_subscription(
            Bool,
            f'/robot_{self.robot_id}/lane_follower/resume',
            self._resume_callback,
            10)

        # IR 센서 캐시 [left, center, right] — 0=도로, 1=흰색라인
        self._ir = [0, 0, 0]
        self._ir_stamp = 0.0   # 마지막 수신 시각 (monotonic)

        self.get_logger().info(
            f'[robot_{self.robot_id}] LaneFollower 시작 '
            f'({self.img_w}×{self.img_h})'
        )

    # ── 파라미터 로드 ─────────────────────────────────────────────

    def _load_params(self):
        g = self.get_parameter
        self.img_w = g('image_width').value
        self.img_h = g('image_height').value

        # BEV 변환 행렬 계산
        src = np.float32(g('bev_src_pts').value).reshape(4, 2)
        dst = np.float32(g('bev_dst_pts').value).reshape(4, 2)
        self.M     = cv2.getPerspectiveTransform(src, dst)
        self.M_inv = cv2.getPerspectiveTransform(dst, src)

        # 도로 ROI 마스크 생성
        # BEV 전에 비도로 영역을 블랙아웃하여 주변 환경 오인식 방지
        roi_flat      = g('roi_pts').value
        roi_pts_arr   = np.array(roi_flat, dtype=np.int32).reshape(-1, 2)
        self.roi_mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
        cv2.fillPoly(self.roi_mask, [roi_pts_arr], 255)

        # HSV 임계값
        self.yellow_low  = np.array([
            g('yellow_h_low').value,
            g('yellow_s_low').value,
            g('yellow_v_low').value])
        self.yellow_high = np.array([
            g('yellow_h_high').value,
            g('yellow_s_high').value,
            g('yellow_v_high').value])
        self.white_s_high = g('white_s_high').value
        self.white_v_low  = g('white_v_low').value

        # 슬라이딩 윈도우
        self.n_win   = g('n_windows').value
        self.margin  = g('margin_px').value
        self.min_pix = g('min_pixels').value

        # Pure Pursuit
        self.lh_straight = g('lookahead_straight_m').value
        self.lh_curve    = g('lookahead_curve_m').value
        self.base_speed  = g('base_speed_mps').value
        self.max_steer   = g('max_steering_rad').value
        self.wheelbase   = g('wheelbase_m').value

        # 정지선
        self.stop_roi_y   = g('stop_line_roi_y').value
        self.stop_thresh  = g('stop_line_threshold').value

        # 교차로 감지
        self.inter_min_px     = g('intersection_yellow_min_px').value
        self.inter_aspect_min = g('intersection_aspect_min').value

        # IR 센서 파라미터
        self.ir_stop_min  = g('ir_stop_min_sensors').value
        self.ir_lane_assist = g('ir_lane_assist').value
        self.ir_lane_gain = g('ir_lane_error_gain').value

    # ── 이미지 콜백 ───────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 오류: {e}')
            return

        # 1. RAW 이미지에서 도로/경계 검출
        masked = cv2.bitwise_and(frame, frame, mask=self.roi_mask)
        hsv_raw = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)

        # 노란 중앙선: HSV 필터
        yellow_raw = cv2.inRange(hsv_raw, self.yellow_low, self.yellow_high)

        # 도로 = 어두운 영역 (S<100, V<90)
        road_raw = cv2.inRange(hsv_raw,
                               np.array([0, 0, 0]),
                               np.array([180, 100, 90]))
        # 경계선 = 도로가 아닌 영역의 에지 (흰선, 녹색, 벽 등)
        boundary_raw = cv2.bitwise_not(road_raw)
        boundary_raw = cv2.bitwise_and(boundary_raw, self.roi_mask)
        # 노란선 영역 제외 (별도 처리)
        white_raw = cv2.bitwise_and(boundary_raw, cv2.bitwise_not(yellow_raw))
        # 모폴로지: 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        white_raw = cv2.morphologyEx(white_raw, cv2.MORPH_OPEN, kernel)

        # 2. 검출된 마스크를 BEV 변환
        yellow_mask = cv2.warpPerspective(
            yellow_raw, self.M, (self.img_w, self.img_h))
        white_mask = cv2.warpPerspective(
            white_raw, self.M, (self.img_w, self.img_h))
        _, yellow_mask = cv2.threshold(yellow_mask, 127, 255, cv2.THRESH_BINARY)
        _, white_mask = cv2.threshold(white_mask, 127, 255, cv2.THRESH_BINARY)

        combined = cv2.bitwise_or(yellow_mask, white_mask)

        # 3. 슬라이딩 윈도우
        left_fit, right_fit, detected = self._sliding_window(
            combined, yellow_mask, white_mask)

        # 4. 정지선 감지 — 신뢰도 기반 보수적 융합
        cam_conf = self._camera_confidence(white_mask, yellow_mask)
        ir_conf  = self._ir_confidence()

        # 신뢰도에 따라 각 센서의 유효 픽셀 임계값을 조정
        cam_eff_thresh = self._effective_threshold(self.stop_thresh, cam_conf)
        ir_eff_min     = self._effective_threshold(
            float(self.ir_stop_min), ir_conf)
        ir_eff_min_int = max(1, int(round(ir_eff_min)))  # 최소 1개

        camera_stop = self._detect_stop_line_with_threshold(
            white_mask, cam_eff_thresh)
        ir_stop     = self._detect_stop_line_ir_with_threshold(
            ir_eff_min_int)

        if camera_stop:
            stop_detected = True
            stop_source   = 'CAM'
            stop_conf     = int(cam_conf)
        elif ir_stop:
            stop_detected = True
            stop_source   = 'IR'
            stop_conf     = int(ir_conf)
            self.get_logger().info(
                f'[robot_{self.robot_id}] 정지선: CAM신뢰도={cam_conf:.0f}% '
                f'→ IR 대체 (ir={self._ir}, IR신뢰도={ir_conf:.0f}%)')
        else:
            stop_detected = False
            stop_source   = 'NONE'
            stop_conf     = 0

        # 5. 중앙 편차 계산
        lateral_err, heading_err, curvature = self._calc_errors(
            left_fit, right_fit, detected)

        # 6. Pure Pursuit 제어 (angular_vel: rad/s, speed: m/s)
        import time
        in_suppression = time.monotonic() < self._stop_suppressed_until
        effective_stop = stop_detected and not in_suppression
        angular_vel, speed = self._pure_pursuit(
            lateral_err, heading_err, curvature, effective_stop)

        # 7. cmd_vel 발행
        twist = Twist()
        if self._stop_flag:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
        elif stop_detected and not in_suppression:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self._stop_flag = True
        else:
            twist.linear.x  = float(speed)
            twist.angular.z = float(angular_vel)
        self._cmd_pub.publish(twist)

        # 8. LaneInfo 발행 (양자화)
        lane_info = LaneInfo()
        lane_info.stamp_ms = self._stamp_ms()
        lane_info.lateral_error_norm  = self._norm_to_int8(lateral_err,  1.0)
        lane_info.heading_error_norm  = self._norm_to_int8(heading_err,  math.pi)
        lane_info.curvature_norm      = self._norm_to_int8(curvature,    0.5)
        intersection_detected = self._detect_intersection(yellow_mask)

        flags = 0
        if detected[0] or detected[1]:
            flags |= 1   # FLAG_CENTER_LINE
        if stop_detected:
            flags |= 2   # FLAG_STOP_LINE
        if intersection_detected:
            flags |= 4   # FLAG_INTERSECTION
        lane_info.flags = flags
        self._lane_pub.publish(lane_info)

        # 9-a. 교차로 TrafficSign 이벤트 발행
        if intersection_detected:
            inter_sign = TrafficSign()
            inter_sign.stamp_ms       = self._stamp_ms()
            inter_sign.robot_id       = self.robot_id
            inter_sign.sign_type      = 2  # INTERSECTION
            inter_sign.confidence_pct = 80
            inter_sign.distance_dm    = 0
            self._sign_pub.publish(inter_sign)

        # 9-b. 정지선 TrafficSign (이벤트 기반)
        if stop_detected:
            sign = TrafficSign()
            sign.stamp_ms       = self._stamp_ms()
            sign.robot_id       = self.robot_id
            sign.sign_type      = 1  # STOP_LINE
            sign.confidence_pct = stop_conf   # CAM=90, IR=75
            sign.distance_dm    = 0
            self._sign_pub.publish(sign)

    # ── 슬라이딩 윈도우 ──────────────────────────────────────────

    def _sliding_window(self, combined, yellow_mask, white_mask):
        """
        황색 마스크 → 중앙선(left_fit) / 백색 마스크 → 우측 차로선(right_fit)
        색상별 독립 히스토그램으로 기저점을 분리 — 교차로 구간 좌우 반전 방지.
        """
        h, w = combined.shape

        # ── 색상별 독립 히스토그램으로 기저점 탐색 ─────────────────
        yellow_hist = np.sum(yellow_mask[h // 2:, :], axis=0)
        white_hist  = np.sum(white_mask[h // 2:, :], axis=0)
        mid = w // 2

        # left_base: 황색(중앙선) 좌측 절반 탐색
        if yellow_hist[:mid].max() > 0:
            left_base = int(np.argmax(yellow_hist[:mid]))
        else:
            left_base = mid // 2   # 황색 미검출 fallback

        # right_base: 백색(경계선) 우측 절반 탐색
        if white_hist[mid:].max() > 0:
            right_base = int(np.argmax(white_hist[mid:]) + mid)
        else:
            right_base = mid + (w - mid) // 2   # 백색 미검출 fallback

        win_h = h // self.n_win
        left_x,  left_y  = [], []
        right_x, right_y = [], []
        lx, rx = left_base, right_base

        # 황색/백색 픽셀 좌표 분리 — 각 윈도우에서 해당 색상 픽셀만 사용
        y_nz   = yellow_mask.nonzero()
        ly,   lx_arr = np.array(y_nz[0]),  np.array(y_nz[1])
        w_nz   = white_mask.nonzero()
        wy,   wx_arr = np.array(w_nz[0]),  np.array(w_nz[1])

        for win in range(self.n_win):
            y_low  = h - (win + 1) * win_h
            y_high = h - win * win_h

            # 좌측 윈도우 — 황색 픽셀만
            lx_low, lx_high = lx - self.margin, lx + self.margin
            good_l = ((ly >= y_low) & (ly < y_high) &
                      (lx_arr >= lx_low) & (lx_arr < lx_high)).nonzero()[0]

            # 우측 윈도우 — 백색 픽셀만
            rx_low, rx_high = rx - self.margin, rx + self.margin
            good_r = ((wy >= y_low) & (wy < y_high) &
                      (wx_arr >= rx_low) & (wx_arr < rx_high)).nonzero()[0]

            left_x.extend(lx_arr[good_l]);  left_y.extend(ly[good_l])
            right_x.extend(wx_arr[good_r]); right_y.extend(wy[good_r])

            if len(good_l) > self.min_pix:
                lx = int(np.mean(lx_arr[good_l]))
            if len(good_r) > self.min_pix:
                rx = int(np.mean(wx_arr[good_r]))

        left_det  = len(left_x)  > self.min_pix * 2
        right_det = len(right_y) > self.min_pix * 2

        left_fit = right_fit = None
        if left_det:
            left_fit = np.polyfit(left_y, left_x, 2)
            self._prev_left_fit = left_fit
        elif self._prev_left_fit is not None:
            left_fit = self._prev_left_fit
            left_det = False
        if right_det:
            right_fit = np.polyfit(right_y, right_x, 2)
            self._prev_right_fit = right_fit
        elif self._prev_right_fit is not None:
            right_fit = self._prev_right_fit
            right_det = False

        return left_fit, right_fit, (left_det, right_det)

    def _resume_callback(self, msg):
        """traffic_law_manager로부터 정지 해제 신호 수신"""
        import time
        if msg.data:
            self._stop_flag = False
            self._stop_suppressed_until = time.monotonic() + 3.0  # 3초간 재래치 억제
            self.get_logger().info(
                f'[robot_{self.robot_id}] 정지 해제 신호 수신 (3초 억제)')

    # ── IR 센서 ────────────────────────────────────────────────────

    def _ir_callback(self, msg: Int16MultiArray):
        """TCRT5000 × 3 수신 — [left, center, right], 1=흰색"""
        import time
        if len(msg.data) >= 3:
            self._ir = list(msg.data[:3])
            self._ir_stamp = time.monotonic()

    def _ir_fresh(self) -> bool:
        """IR 데이터가 0.2초 이내로 신선한지 확인"""
        import time
        return (time.monotonic() - self._ir_stamp) < 0.2

    def _detect_stop_line_ir(self) -> bool:
        """
        IR 3개 중 ir_stop_min_sensors 개 이상 흰색 감지 시 정지선 판정.
        정지선: 도로를 가로지르는 굵은 흰색 선 → 여러 센서 동시 반응.
        단순 차선: 한쪽만 반응 → 정지선 아님.
        """
        if not self._ir_fresh():
            return False
        white_count = sum(1 for v in self._ir if v >= 1)
        return white_count >= self.ir_stop_min

    # ── 정지선 감지 ───────────────────────────────────────────────

    def _detect_stop_line(self, white_mask) -> bool:
        """하단 ROI에서 수평 백색 픽셀 밀도로 정지선 감지"""
        roi = white_mask[self.stop_roi_y:, :]
        # 수평 방향 픽셀 합산
        row_sums = np.sum(roi, axis=1) // 255
        return bool(np.any(row_sums > self.stop_thresh))

    def _detect_stop_line_with_threshold(
            self, white_mask, effective_threshold: float) -> bool:
        """신뢰도가 반영된 유효 임계값으로 정지선을 감지한다."""
        roi = white_mask[self.stop_roi_y:, :]
        row_sums = np.sum(roi, axis=1) // 255
        return bool(np.any(row_sums > effective_threshold))

    def _detect_stop_line_ir_with_threshold(
            self, effective_min_sensors: int) -> bool:
        """신뢰도가 반영된 최소 센서 수로 IR 정지선을 감지한다."""
        if not self._ir_fresh():
            return False
        white_count = sum(1 for v in self._ir if v >= 1)
        return white_count >= effective_min_sensors

    def _detect_intersection(self, yellow_mask) -> bool:
        """
        교차로 감지: 황색 중앙선이 BEV 이미지 중앙부에서
        수평/교차 패턴으로 나타나면 교차로로 판단.
        - 중간 ROI(이미지 세로 25%~75%)에서 황색 픽셀 수가 임계값 초과
        - 해당 영역의 황색 픽셀 bounding box 종횡비가 수평에 가까울 때
        """
        h, w = yellow_mask.shape
        roi = yellow_mask[h // 4 : h * 3 // 4, :]
        px_count = int(np.sum(roi) // 255)
        if px_count < self.inter_min_px:
            return False
        coords = np.column_stack(np.where(roi > 0))
        if len(coords) < 10:
            return False
        y_span = float(coords[:, 0].max() - coords[:, 0].min() + 1)
        x_span = float(coords[:, 1].max() - coords[:, 1].min() + 1)
        aspect = y_span / (x_span + 1e-6)
        return aspect < self.inter_aspect_min

    # ── 중앙 편차 계산 ────────────────────────────────────────────

    def _calc_errors(self, left_fit, right_fit, detected):
        h, w = self.img_h, self.img_w
        y_eval = float(h - 1)
        center_x = w / 2.0

        if left_fit is not None and right_fit is not None:
            lx = np.polyval(left_fit,  y_eval)
            rx = np.polyval(right_fit, y_eval)
            road_center = (lx + rx) / 2.0
        elif left_fit is not None:
            lx = np.polyval(left_fit, y_eval)
            road_center = lx + w * 0.25  # 차로폭 추정
        elif right_fit is not None:
            rx = np.polyval(right_fit, y_eval)
            road_center = rx - w * 0.25
        else:
            # 카메라 차선 전실 → IR 보조 조향
            if self.ir_lane_assist and self._ir_fresh():
                left_v, center_v, right_v = self._ir
                ir_conf_ratio = self._ir_confidence() / 100.0
                gain = self.ir_lane_gain * ir_conf_ratio
                if left_v and not right_v:
                    # 좌측 IR만 감지 = 황색 중앙선에 너무 근접 → 우측 보정
                    ir_err = -gain   # 신뢰도 낮으면 보정 게인도 감소
                elif right_v and not left_v:
                    # 우측 IR만 감지 = 흰색 경계선에 너무 근접 → 좌측 보정
                    ir_err = +gain
                else:
                    ir_err = 0.0
                self.get_logger().debug(
                    f'[IR 차선 보조] ir={self._ir} err={ir_err:.2f}')
                return float(ir_err), 0.0, 0.0
            return 0.0, 0.0, 0.0

        # lateral_error: 화면 중앙 대비 (정규화 -1~1)
        lateral_err = (road_center - center_x) / (w / 2.0)
        lateral_err = float(np.clip(lateral_err, -1.0, 1.0))

        # heading_error: 차선 기울기에서 추정
        heading_err = 0.0
        if left_fit is not None:
            dy  = 50.0
            lx1 = np.polyval(left_fit, y_eval)
            lx2 = np.polyval(left_fit, y_eval - dy)
            heading_err = math.atan2(lx2 - lx1, dy)
        elif right_fit is not None:
            dy  = 50.0
            rx1 = np.polyval(right_fit, y_eval)
            rx2 = np.polyval(right_fit, y_eval - dy)
            heading_err = math.atan2(rx2 - rx1, dy)
        heading_err = float(np.clip(heading_err, -math.pi, math.pi))

        # 곡률 (2차 계수 기반)
        curvature = 0.0
        if left_fit is not None:
            curvature = abs(left_fit[0]) * 2.0
        elif right_fit is not None:
            curvature = abs(right_fit[0]) * 2.0
        curvature = float(np.clip(curvature, 0.0, 0.5))

        return lateral_err, heading_err, curvature

    # ── Pure Pursuit 제어 ─────────────────────────────────────────

    def _pure_pursuit(self, lateral_err, heading_err, curvature, stop_detected):
        """
        Pure Pursuit 조향각 계산 (정확한 기하학적 공식 적용)
        - lookahead distance: 곡률에 따라 동적 조정
        - alpha: lookahead point까지의 실제 각도 (lateral + heading 합산)
        - steering = atan2(2 * L * sin(alpha), lookahead)
        """
        if stop_detected:
            return 0.0, 0.0

        curvature_norm = min(curvature / 0.5, 1.0)
        lookahead = self.lh_straight - curvature_norm * (
            self.lh_straight - self.lh_curve)
        lookahead = max(lookahead, 0.05)  # 최소값 보호

        # lookahead point까지의 실제 각도 계산
        # lateral_err (정규화 -1~1) → 실제 횡방향 거리 (m) 추정
        lateral_m = lateral_err * (self.wheelbase * 2.0)
        alpha = math.atan2(lateral_m, lookahead) + heading_err * 0.5
        alpha = float(np.clip(alpha, -math.pi / 2, math.pi / 2))

        # Pure Pursuit 조향각 (rad)
        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha), lookahead)
        steering = float(np.clip(steering, -self.max_steer, self.max_steer))

        # 곡률 + 조향각이 클수록 속도 감소
        steer_ratio = abs(steering) / self.max_steer
        speed = self.base_speed * (1.0 - curvature_norm * 0.4) * (
            1.0 - steer_ratio * 0.3)
        speed = max(speed, self.base_speed * 0.3)

        # 조향각(rad) → 차동구동 각속도(rad/s) 변환
        # ω = v · tan(δ) / L
        angular_vel = speed * math.tan(steering) / self.wheelbase

        return angular_vel, speed

    # ── 유틸 ──────────────────────────────────────────────────────

    @staticmethod
    def _norm_to_int8(value: float, max_val: float) -> int:
        """float 값을 int8 (-128~127)로 양자화"""
        normalized = value / max_val
        return int(max(-128, min(127, normalized * 127)))

    def _effective_threshold(self, base: float, conf: float) -> float:
        """
        신뢰도(0~100)에 따라 기본 임계값을 조정한다.
        보수적 원칙: 신뢰도 낮을수록 임계값 증가 → 더 강한 신호 필요.
        신뢰도 최솟값 30% 보장 (센서 완전 비활성화 방지).
        """
        conf_clamped = max(30.0, min(100.0, conf))
        return base / (conf_clamped / 100.0)

    def _camera_confidence(self, white_mask, yellow_mask) -> float:
        """
        카메라 신뢰도 계산 (40~100).
        차선 픽셀 밀도와 황색/백색 검출 여부로 판단.
        조명 불량 또는 차선 미검출 시 신뢰도 하락.
        """
        h, w = white_mask.shape
        expected_px = w * h * 0.01   # BEV 도로 영역 차선 픽셀 현실적 기준 (약 3,000px)
        white_px  = float(np.sum(white_mask)  // 255)
        yellow_px = float(np.sum(yellow_mask) // 255)
        total_px  = white_px + yellow_px

        # 픽셀 비율 → 0~1
        ratio = min(total_px / max(expected_px, 1.0), 1.0)
        # 40(최소) ~ 100(최대) 선형 매핑
        conf = 40.0 + ratio * 60.0
        return float(conf)

    def _ir_confidence(self) -> float:
        """
        IR 센서 신뢰도 계산 (50~90).
        신선도: 0.2초 이내 데이터면 기본 신뢰도 85.
        신선도 초과 시 시간에 따라 지수 감쇠.
        """
        import time, math as _math
        if self._ir_stamp == 0.0:
            return 50.0   # IR 데이터 미수신 상태 — 최저 신뢰도
        age = time.monotonic() - self._ir_stamp
        if age > 1.0:
            return 50.0   # 1초 초과 → 최저 신뢰도
        # 지수 감쇠: tau=0.3s
        base_conf = 85.0
        decayed   = base_conf * _math.exp(-age / 0.3)
        return float(max(50.0, decayed))

    def _stamp_ms(self) -> int:
        ns = self.get_clock().now().nanoseconds
        return int((ns // 1_000_000) & 0xFFFFFFFF)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
