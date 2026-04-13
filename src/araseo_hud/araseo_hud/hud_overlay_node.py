#!/usr/bin/env python3
"""
ARASEO-DALIMI HUD Overlay Node
카메라 피드 위에 한국 도로교통법 기반 주행 HUD를 오버레이하여 발행.

Layer 1: 반대편 차선 (주행 금지 영역)
Layer 2: 주행 가능 영역 (우측 차선)
Layer 3: 차선 마킹 (황색 중앙선, 흰색 경계선)
Layer 4: 주행 경로 (점선 + 방향 화살표)
Layer 5: 웨이포인트 마커 (화면 내/외)
Layer 6: 장애물 박스
Layer 7: HUD 패널 (상태 정보 텍스트)
"""
import math
import time
from enum import IntEnum

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from araseo_dalimi_interfaces.msg import (
    LaneInfo, TrafficSign, FleetStatus, MissionStatus
)


# ── 교통법규 상태 (traffic_law_manager_node 와 동일) ──────────────
class DrivingState(IntEnum):
    NORMAL           = 0
    APPROACHING_STOP = 1
    STOP_HOLD        = 2
    INTERSECTION     = 3
    YIELD            = 4
    FOLLOWING        = 5
    COLLISION_AVOID  = 6


_STATE_NAMES = {
    0: 'NORMAL',
    1: 'APPROACHING_STOP',
    2: 'STOP_HOLD',
    3: 'INTERSECTION',
    4: 'YIELD',
    5: 'FOLLOWING',
    6: 'COLLISION_AVOID',
}

_SIGN_NAMES = {0: 'NONE', 1: 'STOP_LINE', 2: 'INTERSECTION', 3: 'SPEED_LIMIT'}

# ── BEV 상수 ─────────────────────────────────────────────────────
BEV_W, BEV_H = 640, 480


class HudOverlayNode(Node):

    def __init__(self):
        super().__init__('hud_overlay')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameters('', [
            ('robot_id',            0),
            ('road_width_mm',       280.0),
            ('lane_width_mm',       140.0),
            ('camera_height_mm',    120.0),
            ('show_drivable_area',  True),
            ('show_lane_markings',  True),
            ('show_path',           True),
            ('show_waypoints',      True),
            ('show_obstacles',      True),
            ('show_no_entry_zone',  True),
            ('show_traffic_state',  True),
            ('alpha_drivable',      0.30),
            ('alpha_no_entry',      0.18),
            ('publish_hz',          20.0),
            # BEV 파라미터 (lane_params.yaml 에서 로드)
            ('bev_src_pts', [100, 480, 540, 480, 220, 300, 420, 300]),
            ('bev_dst_pts', [100, 480, 540, 480, 100,   0, 540,   0]),
            ('image_width',  640),
            ('image_height', 480),
            # 교통법규 거리 (traffic_law_params 에서 로드)
            ('follow_distance_mm',  400),
            ('collision_stop_mm',   200),
            ('yield_distance_mm',   500),
        ])

        g = self.get_parameter
        self.robot_id       = g('robot_id').value
        self.road_width     = g('road_width_mm').value
        self.lane_width     = g('lane_width_mm').value
        self.cam_height     = g('camera_height_mm').value
        self.show_drivable  = g('show_drivable_area').value
        self.show_markings  = g('show_lane_markings').value
        self.show_path      = g('show_path').value
        self.show_wp        = g('show_waypoints').value
        self.show_obstacles = g('show_obstacles').value
        self.show_no_entry  = g('show_no_entry_zone').value
        self.show_state     = g('show_traffic_state').value
        self.alpha_drive    = g('alpha_drivable').value
        self.alpha_no_entry = g('alpha_no_entry').value
        self.img_w          = g('image_width').value
        self.img_h          = g('image_height').value
        self.follow_dist_mm = g('follow_distance_mm').value
        self.collision_mm   = g('collision_stop_mm').value
        self.yield_dist_mm  = g('yield_distance_mm').value

        # BEV 행렬
        src = np.float32(g('bev_src_pts').value).reshape(4, 2)
        dst = np.float32(g('bev_dst_pts').value).reshape(4, 2)
        self.M     = cv2.getPerspectiveTransform(src, dst)
        self.M_inv = cv2.getPerspectiveTransform(dst, src)

        # px_per_mm: BEV 해상도와 도로 폭으로 계산
        self.px_per_mm = 440.0 / 280.0

        self._bridge = CvBridge()

        # ── 캐시 ──────────────────────────────────────────────────
        self._lane_info   = None
        self._cmd_vel     = None
        self._latest_sign = None
        self._fleet       = None
        self._mission     = None
        self._my_x_mm     = 0
        self._my_y_mm     = 0
        self._my_theta    = 0
        self._drive_state = DrivingState.NORMAL
        self._tick        = 0  # 점멸용 카운터

        # ── 구독 ──────────────────────────────────────────────────
        rid = self.robot_id
        self.create_subscription(
            Image, f'/robot_{rid}/camera/image_raw',
            self._image_cb, 1)
        self.create_subscription(
            LaneInfo, f'/robot_{rid}/lane_info',
            self._lane_cb, 10)
        self.create_subscription(
            Twist, f'/robot_{rid}/cmd_vel_safe',
            self._cmd_cb, 10)
        self.create_subscription(
            TrafficSign, f'/robot_{rid}/traffic_sign',
            self._sign_cb, 10)
        self.create_subscription(
            FleetStatus, '/fleet/status',
            self._fleet_cb, 10)
        self.create_subscription(
            MissionStatus, f'/robot_{rid}/mission_status',
            self._mission_cb, 10)

        # ── 발행 ──────────────────────────────────────────────────
        self._hud_pub = self.create_publisher(
            Image, f'/robot_{rid}/camera/hud_view', 10)

        self.get_logger().info(
            f'[robot_{rid}] HUD Overlay 시작 ({self.img_w}x{self.img_h})')

    # ── 구독 콜백 ─────────────────────────────────────────────────

    def _lane_cb(self, msg: LaneInfo):
        self._lane_info = msg

    def _cmd_cb(self, msg: Twist):
        self._cmd_vel = msg

    def _sign_cb(self, msg: TrafficSign):
        self._latest_sign = msg

    def _fleet_cb(self, msg: FleetStatus):
        self._fleet = msg
        for i, rid in enumerate(msg.robot_ids):
            if rid == self.robot_id:
                self._my_x_mm  = msg.x_mm[i]
                self._my_y_mm  = msg.y_mm[i]
                self._my_theta = msg.theta_mrad[i]
                break

    def _mission_cb(self, msg: MissionStatus):
        self._mission = msg

    # ── 좌표 변환 ─────────────────────────────────────────────────

    def _road_to_pixel(self, x_mm: float, z_mm: float):
        """
        도로 좌표 (x_mm, z_mm) → 카메라 픽셀 (u, v) 또는 None.
        BEV 좌표계: 하단 중앙 = 로봇, x=좌우, z=전방(위).
        """
        bev_cx = BEV_W / 2.0
        bev_u = bev_cx + x_mm * self.px_per_mm
        bev_v = BEV_H - z_mm * self.px_per_mm

        if bev_u < -BEV_W or bev_u > BEV_W * 2 or bev_v < -BEV_H or bev_v > BEV_H * 2:
            return None

        pt_bev = np.float32([[[bev_u, bev_v]]])
        pt_cam = cv2.perspectiveTransform(pt_bev, self.M_inv)
        u, v = float(pt_cam[0][0][0]), float(pt_cam[0][0][1])

        if 0 <= u < self.img_w and 0 <= v < self.img_h:
            return int(u), int(v)
        return None

    def _road_to_pixel_raw(self, x_mm: float, z_mm: float):
        """화면 밖도 좌표를 반환 (edge arrow 용)."""
        bev_cx = BEV_W / 2.0
        bev_u = bev_cx + x_mm * self.px_per_mm
        bev_v = BEV_H - z_mm * self.px_per_mm
        pt_bev = np.float32([[[bev_u, bev_v]]])
        pt_cam = cv2.perspectiveTransform(pt_bev, self.M_inv)
        return float(pt_cam[0][0][0]), float(pt_cam[0][0][1])

    def _boundary_pts(self, x_offset_mm, z_start_mm, z_end_mm, n=20):
        """도로 경계선 포인트 배열 생성 (화면 내 점만)."""
        pts = []
        for i in range(n):
            z = z_start_mm + (z_end_mm - z_start_mm) * i / (n - 1)
            p = self._road_to_pixel(x_offset_mm, z)
            if p is not None:
                pts.append(p)
        return pts

    def _boundary_pts_all(self, x_offset_mm, z_start_mm, z_end_mm, n=20):
        """화면 밖 점도 포함하여 반환."""
        pts = []
        for i in range(n):
            z = z_start_mm + (z_end_mm - z_start_mm) * i / (n - 1)
            bev_cx = BEV_W / 2.0
            bev_u = bev_cx + x_offset_mm * self.px_per_mm
            bev_v = BEV_H - z * self.px_per_mm
            pt_bev = np.float32([[[bev_u, bev_v]]])
            pt_cam = cv2.perspectiveTransform(pt_bev, self.M_inv)
            u, v = int(pt_cam[0][0][0]), int(pt_cam[0][0][1])
            pts.append((u, v))
        return pts

    # ── 드로잉 유틸 ───────────────────────────────────────────────

    @staticmethod
    def _fill_poly_alpha(frame, pts, color, alpha):
        """반투명 폴리곤 채우기."""
        if len(pts) < 3:
            return
        overlay = frame.copy()
        arr = np.array(pts, dtype=np.int32)
        cv2.fillPoly(overlay, [arr], color)
        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)

    @staticmethod
    def _draw_dashed_polyline(frame, pts, color, thickness=2, gap=12):
        """점선 폴리라인."""
        for i in range(len(pts) - 1):
            p0, p1 = pts[i], pts[i + 1]
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            segs = max(int(dist / gap), 1)
            for s in range(segs):
                if s % 2 != 0:
                    continue
                t0 = s / segs
                t1 = min((s + 1) / segs, 1.0)
                a = (int(p0[0] + t0 * (p1[0] - p0[0])),
                     int(p0[1] + t0 * (p1[1] - p0[1])))
                b = (int(p0[0] + t1 * (p1[0] - p0[0])),
                     int(p0[1] + t1 * (p1[1] - p0[1])))
                cv2.line(frame, a, b, color, thickness, cv2.LINE_AA)

    @staticmethod
    def _draw_arrow_head(frame, pt, direction, size, color, thickness=2):
        """방향 삼각형 화살표."""
        dx = math.cos(direction)
        dy = math.sin(direction)
        tip = (int(pt[0] + dx * size), int(pt[1] + dy * size))
        left  = (int(pt[0] - dy * size * 0.5 - dx * size * 0.3),
                 int(pt[1] + dx * size * 0.5 - dy * size * 0.3))
        right = (int(pt[0] + dy * size * 0.5 - dx * size * 0.3),
                 int(pt[1] - dx * size * 0.5 - dy * size * 0.3))
        tri = np.array([tip, left, right], dtype=np.int32)
        cv2.fillPoly(frame, [tri], color)

    @staticmethod
    def _draw_text_bg(frame, text, org, font_scale, fg, bg,
                      thickness=1, pad=4):
        """배경색이 있는 텍스트."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), bl = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = org
        cv2.rectangle(frame, (x - pad, y - th - pad),
                       (x + tw + pad, y + bl + pad), bg, -1)
        cv2.putText(frame, text, (x, y), font, font_scale, fg,
                    thickness, cv2.LINE_AA)

    def _draw_edge_arrow(self, frame, target_u, target_v, color, label=''):
        """화면 밖 대상에 가장자리 화살표 + 배지."""
        cx, cy = self.img_w // 2, self.img_h // 2
        dx = target_u - cx
        dy = target_v - cy
        dist = math.hypot(dx, dy)
        if dist < 1:
            return

        margin = 30
        # 화면 경계까지 스케일
        scale_x = (self.img_w // 2 - margin) / max(abs(dx), 1)
        scale_y = (self.img_h // 2 - margin) / max(abs(dy), 1)
        scale = min(scale_x, scale_y)
        ex = int(cx + dx * scale)
        ey = int(cy + dy * scale)

        angle = math.atan2(dy, dx)
        self._draw_arrow_head(frame, (ex, ey), angle, 14, color, 2)

        if label:
            lx = max(5, min(ex - 10, self.img_w - 60))
            ly = max(15, min(ey - 18, self.img_h - 5))
            self._draw_text_bg(frame, label, (lx, ly), 0.35, (255, 255, 255),
                               color, 1, 2)

    # ── 상태 추론 ─────────────────────────────────────────────────

    def _infer_drive_state(self) -> DrivingState:
        """cmd_vel + lane_info + sign 으로 현재 주행 상태를 추론."""
        vel = 0.0
        if self._cmd_vel is not None:
            vel = self._cmd_vel.linear.x

        li = self._lane_info
        sign = self._latest_sign

        # 충돌 회피: 속도 0 + 전방 근접 로봇
        if vel == 0.0 and self._fleet is not None:
            my_theta = self._my_theta * 0.001
            for i, rid in enumerate(self._fleet.robot_ids):
                if rid == self.robot_id:
                    continue
                dx = self._fleet.x_mm[i] - self._my_x_mm
                dy = self._fleet.y_mm[i] - self._my_y_mm
                d = math.sqrt(dx * dx + dy * dy)
                if d < self.collision_mm:
                    ang = math.atan2(dy, dx)
                    diff = abs(math.atan2(math.sin(ang - my_theta),
                                          math.cos(ang - my_theta)))
                    if diff < math.pi / 4.0:
                        return DrivingState.COLLISION_AVOID

        # 정지선 / 교차로
        if sign is not None:
            if sign.sign_type == 1:  # STOP_LINE
                if vel == 0.0:
                    return DrivingState.STOP_HOLD
                return DrivingState.APPROACHING_STOP
            if sign.sign_type == 2:  # INTERSECTION
                return DrivingState.INTERSECTION

        if li is not None and bool(li.flags & 4):
            return DrivingState.INTERSECTION

        # 전방 추종
        if self._fleet is not None and vel > 0:
            my_theta = self._my_theta * 0.001
            for i, rid in enumerate(self._fleet.robot_ids):
                if rid == self.robot_id:
                    continue
                dx = self._fleet.x_mm[i] - self._my_x_mm
                dy = self._fleet.y_mm[i] - self._my_y_mm
                d = math.sqrt(dx * dx + dy * dy)
                if d > self.follow_dist_mm:
                    continue
                ang = math.atan2(dy, dx)
                diff = abs(math.atan2(math.sin(ang - my_theta),
                                      math.cos(ang - my_theta)))
                if diff < math.pi / 6.0:
                    return DrivingState.FOLLOWING

        return DrivingState.NORMAL

    # ── Layer 드로잉 ──────────────────────────────────────────────

    def _draw_layer1_no_entry(self, frame):
        """Layer 1 — 반대편 차선 (주행 금지)."""
        if not self.show_no_entry:
            return
        z0, z1 = 50.0, 600.0
        left_outer = self._boundary_pts_all(-self.lane_width * 2, z0, z1, 20)
        left_inner = self._boundary_pts_all(-8.0, z0, z1, 20)

        poly = left_inner + list(reversed(left_outer))
        # 화면 내로 클리핑
        clipped = [(max(0, min(self.img_w - 1, u)),
                     max(0, min(self.img_h - 1, v))) for u, v in poly]
        self._fill_poly_alpha(frame, clipped, (50, 50, 220), self.alpha_no_entry)

        # "반대편" 텍스트
        mid_pt = self._road_to_pixel(-self.lane_width, 200.0)
        if mid_pt:
            cv2.putText(frame, '<- No Entry', (mid_pt[0] - 30, mid_pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (100, 100, 220), 1, cv2.LINE_AA)

    def _draw_layer2_drivable(self, frame):
        """Layer 2 — 주행 가능 영역 (우측 차선)."""
        if not self.show_drivable:
            return
        z0, z1 = 50.0, 600.0
        right_inner = self._boundary_pts_all(8.0, z0, z1, 20)
        right_outer = self._boundary_pts_all(self.lane_width - 8.0, z0, z1, 20)

        poly = right_inner + list(reversed(right_outer))
        clipped = [(max(0, min(self.img_w - 1, u)),
                     max(0, min(self.img_h - 1, v))) for u, v in poly]

        # lateral_error 경고 색상
        color = (90, 210, 0)  # 녹색
        if self._lane_info is not None:
            lat = abs(self._lane_info.lateral_error_norm / 127.0)
            if lat > 0.65:
                color = (0, 165, 255)  # 주황 경고
        self._fill_poly_alpha(frame, clipped, color, self.alpha_drive)

    def _draw_layer3_markings(self, frame):
        """Layer 3 — 차선 마킹."""
        if not self.show_markings:
            return
        z0, z1 = 50.0, 600.0
        # 황색 이중 중앙선
        center1 = self._boundary_pts(0.0, z0, z1, 25)
        center2 = self._boundary_pts(-6.0, z0, z1, 25)
        if len(center1) > 1:
            cv2.polylines(frame, [np.array(center1, np.int32)], False,
                          (0, 255, 255), 3, cv2.LINE_AA)
        if len(center2) > 1:
            cv2.polylines(frame, [np.array(center2, np.int32)], False,
                          (0, 255, 255), 3, cv2.LINE_AA)

        # 흰색 우측 경계선
        white_line = self._boundary_pts(self.lane_width, z0, z1, 25)
        if len(white_line) > 1:
            cv2.polylines(frame, [np.array(white_line, np.int32)], False,
                          (255, 255, 255), 3, cv2.LINE_AA)

        # 반대편 외측 경계선
        opp_line = self._boundary_pts(-self.lane_width * 2, z0, z1, 25)
        if len(opp_line) > 1:
            cv2.polylines(frame, [np.array(opp_line, np.int32)], False,
                          (100, 100, 100), 1, cv2.LINE_AA)

    def _draw_layer4_path(self, frame):
        """Layer 4 — 주행 경로."""
        if not self.show_path:
            return
        state = self._drive_state
        z0, z1 = 50.0, 600.0

        # 상태별 색상
        path_color_map = {
            DrivingState.NORMAL:           (255, 170, 0),
            DrivingState.FOLLOWING:        (255, 170, 0),
            DrivingState.APPROACHING_STOP: (0, 220, 255),
            DrivingState.STOP_HOLD:        (0, 0, 220),
            DrivingState.INTERSECTION:     (0, 140, 255),
            DrivingState.YIELD:            (0, 140, 255),
            DrivingState.COLLISION_AVOID:  (0, 0, 220),
        }
        color = path_color_map.get(state, (255, 170, 0))

        # 점멸: YIELD, COLLISION_AVOID
        blink = (state in (DrivingState.YIELD, DrivingState.COLLISION_AVOID)
                 and self._tick % 10 < 5)

        # STOP_HOLD: 경로 숨김
        if state == DrivingState.STOP_HOLD:
            return
        if blink:
            return

        path_pts = self._boundary_pts(self.lane_width / 2.0, z0, z1, 25)
        if len(path_pts) > 1:
            self._draw_dashed_polyline(frame, path_pts, color, 2, 14)

        # 방향 화살표 (4개)
        for i in range(0, len(path_pts) - 1, max(len(path_pts) // 4, 1)):
            p0 = path_pts[i]
            p1 = path_pts[min(i + 1, len(path_pts) - 1)]
            if p0 == p1:
                continue
            angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            self._draw_arrow_head(frame, p0, angle, 8, color)

    def _draw_layer5_waypoints(self, frame):
        """Layer 5 — 웨이포인트 마커."""
        if not self.show_wp or self._mission is None:
            return

        mission = self._mission
        goal_idx = mission.goal_idx if hasattr(mission, 'goal_idx') else 0
        total = mission.total_goals if hasattr(mission, 'total_goals') else 0

        if not hasattr(mission, 'goal_x_mm') or not hasattr(mission, 'goal_y_mm'):
            return

        goals_x = list(mission.goal_x_mm) if hasattr(mission, 'goal_x_mm') else []
        goals_y = list(mission.goal_y_mm) if hasattr(mission, 'goal_y_mm') else []

        my_theta_rad = self._my_theta * 0.001

        for gi in range(len(goals_x)):
            gx = goals_x[gi]
            gy = goals_y[gi]

            # 글로벌 → 로봇 로컬 좌표 변환
            dx = gx - self._my_x_mm
            dy = gy - self._my_y_mm
            # 로봇 전방(z) / 좌우(x) 로 회전
            cos_t = math.cos(-my_theta_rad)
            sin_t = math.sin(-my_theta_rad)
            local_x = dx * cos_t - dy * sin_t
            local_z = dx * sin_t + dy * cos_t

            dist_mm = math.sqrt(dx * dx + dy * dy)

            # 색상
            if gi == goal_idx:
                wp_color = (0, 0, 255)    # 빨강 (현재)
            elif gi == goal_idx + 1:
                wp_color = (0, 128, 255)  # 주황 (다음)
            else:
                wp_color = (100, 100, 100)  # 회색

            pt = self._road_to_pixel(local_x, local_z)
            if pt is not None:
                # 화면 내: 원형 마커
                radius = max(5, min(20, int(3000 / max(dist_mm, 50))))
                cv2.circle(frame, pt, radius, wp_color, 2, cv2.LINE_AA)
                cv2.circle(frame, pt, 3, wp_color, -1)
                label = f'WP{gi}'
                dist_str = f'{dist_mm / 10:.0f}cm'
                cv2.putText(frame, label, (pt[0] + radius + 3, pt[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, wp_color,
                            1, cv2.LINE_AA)
                cv2.putText(frame, dist_str, (pt[0] + radius + 3, pt[1] + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.30, wp_color,
                            1, cv2.LINE_AA)
            else:
                # 화면 밖: 가장자리 화살표
                raw_u, raw_v = self._road_to_pixel_raw(local_x, local_z)
                dist_str = f'WP{gi} {dist_mm / 10:.0f}cm'
                self._draw_edge_arrow(frame, raw_u, raw_v, wp_color, dist_str)

    def _draw_layer6_obstacles(self, frame):
        """Layer 6 — 장애물 박스 (타 로봇)."""
        if not self.show_obstacles or self._fleet is None:
            return

        my_theta_rad = self._my_theta * 0.001

        for i, rid in enumerate(self._fleet.robot_ids):
            if rid == self.robot_id:
                continue

            gx = self._fleet.x_mm[i]
            gy = self._fleet.y_mm[i]
            dx = gx - self._my_x_mm
            dy = gy - self._my_y_mm
            dist = math.sqrt(dx * dx + dy * dy)

            # 로봇 로컬 좌표
            cos_t = math.cos(-my_theta_rad)
            sin_t = math.sin(-my_theta_rad)
            local_x = dx * cos_t - dy * sin_t
            local_z = dx * sin_t + dy * cos_t

            # 전방 판단
            angle_to = math.atan2(dy, dx)
            angle_diff = abs(math.atan2(
                math.sin(angle_to - my_theta_rad),
                math.cos(angle_to - my_theta_rad)))

            # 위험 등급 판단
            box_color = None
            label = ''
            if dist < self.collision_mm and angle_diff < math.pi / 4.0:
                box_color = (0, 0, 255)
                label = 'COLLISION'
            elif dist < self.follow_dist_mm and angle_diff < math.pi / 6.0:
                box_color = (0, 140, 255)
                label = 'AHEAD'
            elif dist < self.yield_dist_mm:
                # 우측 방향 체크
                right_angle = my_theta_rad - math.pi / 2.0
                yield_diff = abs(math.atan2(
                    math.sin(angle_to - right_angle),
                    math.cos(angle_to - right_angle)))
                if yield_diff < math.pi / 4.0:
                    box_color = (0, 220, 255)
                    label = 'YIELD'

            if box_color is None:
                continue

            pt = self._road_to_pixel(local_x, local_z)
            if pt is not None:
                half = max(8, min(30, int(2000 / max(dist, 50))))
                cv2.rectangle(frame,
                              (pt[0] - half, pt[1] - half),
                              (pt[0] + half, pt[1] + half),
                              box_color, 2)
                self._draw_text_bg(frame, label,
                                   (pt[0] - half, pt[1] - half - 14),
                                   0.35, (255, 255, 255), box_color, 1, 2)
            else:
                raw_u, raw_v = self._road_to_pixel_raw(local_x, local_z)
                self._draw_edge_arrow(frame, raw_u, raw_v, box_color,
                                      f'R{rid} {label}')

    def _draw_layer7_hud_panel(self, frame):
        """Layer 7 — HUD 정보 패널."""
        h, w = frame.shape[:2]

        # ── 우상단 패널 ───────────────────────────────────────────
        panel_w, panel_h = 185, 100
        px, py = w - panel_w - 8, 8

        overlay = frame.copy()
        cv2.rectangle(overlay, (px, py), (px + panel_w, py + panel_h),
                      (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = 0.38
        clr = (220, 220, 220)
        y0 = py + 16
        dy = 16

        cv2.putText(frame, f'Robot {self.robot_id}', (px + 5, y0),
                    font, 0.45, (0, 220, 255), 1, cv2.LINE_AA)
        y0 += dy

        speed = 0.0
        steer = 0.0
        if self._cmd_vel:
            speed = self._cmd_vel.linear.x
            steer = self._cmd_vel.angular.z
        cv2.putText(frame, f'SPD {speed:.3f} m/s', (px + 5, y0),
                    font, fs, clr, 1, cv2.LINE_AA)
        y0 += dy
        cv2.putText(frame, f'STR {math.degrees(steer):+.1f} deg',
                    (px + 5, y0), font, fs, clr, 1, cv2.LINE_AA)
        y0 += dy

        state_name = _STATE_NAMES.get(int(self._drive_state), '?')
        state_color = clr
        if self._drive_state == DrivingState.STOP_HOLD:
            state_color = (0, 0, 255)
        elif self._drive_state == DrivingState.COLLISION_AVOID:
            state_color = (0, 0, 255)
        elif self._drive_state in (DrivingState.YIELD, DrivingState.INTERSECTION):
            state_color = (0, 180, 255)
        cv2.putText(frame, f'STATE: {state_name}', (px + 5, y0),
                    font, fs, state_color, 1, cv2.LINE_AA)
        y0 += dy

        # 웨이포인트 목록 (최대 3개)
        if self._mission is not None and hasattr(self._mission, 'goal_idx'):
            gi = self._mission.goal_idx
            total = self._mission.total_goals if hasattr(self._mission, 'total_goals') else 0
            cv2.putText(frame, f'WP {gi}/{total}', (px + 5, y0),
                        font, fs, (180, 220, 255), 1, cv2.LINE_AA)

        # ── 좌상단 패널 (센서 상태) ──────────────────────────────
        lp_w, lp_h = 175, 80
        lx, ly = 8, 8

        overlay2 = frame.copy()
        cv2.rectangle(overlay2, (lx, ly), (lx + lp_w, ly + lp_h),
                      (0, 0, 0), -1)
        cv2.addWeighted(overlay2, 0.55, frame, 0.45, 0, frame)

        y0 = ly + 16
        flags = self._lane_info.flags if self._lane_info else 0

        lane_det = bool(flags & 1)
        stop_det = bool(flags & 2)
        inter_det = bool(flags & 4)

        def status_color(det):
            return (0, 255, 0) if det else (0, 0, 180)

        cv2.putText(frame, f'LANE: {"DETECTED" if lane_det else "---"}',
                    (lx + 5, y0), font, fs, status_color(lane_det),
                    1, cv2.LINE_AA)
        y0 += dy
        cv2.putText(frame, f'STOP: {"DETECTED" if stop_det else "---"}',
                    (lx + 5, y0), font, fs, status_color(stop_det),
                    1, cv2.LINE_AA)
        y0 += dy
        cv2.putText(frame, f'INTER: {"DETECTED" if inter_det else "---"}',
                    (lx + 5, y0), font, fs, status_color(inter_det),
                    1, cv2.LINE_AA)
        y0 += dy

        sign_name = 'NONE'
        if self._latest_sign:
            sign_name = _SIGN_NAMES.get(self._latest_sign.sign_type, '?')
        cv2.putText(frame, f'SIGN: {sign_name}', (lx + 5, y0),
                    font, fs, (200, 200, 100), 1, cv2.LINE_AA)

        # ── 하단 중앙 (미션 진행률) ──────────────────────────────
        if self._mission is not None:
            gi = getattr(self._mission, 'goal_idx', 0)
            total = getattr(self._mission, 'total_goals', 0)
            pct = getattr(self._mission, 'progress_percent', 0.0)

            text = f'Mission: {gi}/{total}  {pct:.0f}%'
            (tw, th), _ = cv2.getTextSize(text, font, 0.45, 1)
            mx = (w - tw) // 2
            my = h - 15

            overlay3 = frame.copy()
            cv2.rectangle(overlay3, (mx - 6, my - th - 6),
                          (mx + tw + 6, my + 6), (0, 0, 0), -1)
            cv2.addWeighted(overlay3, 0.5, frame, 0.5, 0, frame)
            cv2.putText(frame, text, (mx, my), font, 0.45,
                        (220, 220, 220), 1, cv2.LINE_AA)

    # ── 이미지 콜백 (메인) ────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 오류: {e}')
            return

        self._tick += 1
        self._drive_state = self._infer_drive_state()

        # Layer 순서대로 오버레이
        self._draw_layer1_no_entry(frame)
        self._draw_layer2_drivable(frame)
        self._draw_layer3_markings(frame)
        self._draw_layer4_path(frame)
        self._draw_layer5_waypoints(frame)
        self._draw_layer6_obstacles(frame)
        if self.show_state:
            self._draw_layer7_hud_panel(frame)

        # 발행
        out_msg = self._bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self._hud_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HudOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
