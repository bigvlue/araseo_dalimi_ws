#!/usr/bin/env python3
"""
Waypoint Follower Node (ARASEO-DALIMI)

Host PC가 발행하는 핑키 위치(/robot_{id}/pose, mm)를 구독하고,
docs/route_map.json의 경로(픽셀)를 Pure Pursuit로 추종해서
/robot_{id}/cmd_vel_safe (Twist)를 발행한다.

웹 UI (포트 8082)에서 경로 1~14 선택 + START/STOP 제어.

실행 전: export ROS_DOMAIN_ID=204
"""
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from araseo_dalimi_interfaces.msg import RobotPose


DEFAULT_ROUTE_MAP = '/home/pinky/araseo_dalimi_ws/docs/route_map.json'


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameters('', [
            ('robot_id',          14),
            ('route_id',          1),
            ('route_map_path',    DEFAULT_ROUTE_MAP),
            ('lookahead_px',      80.0),
            ('base_speed_mps',    0.12),
            ('max_angular_rad',   1.0),
            ('goal_tolerance_px', 30.0),
            ('web_port',          8082),
            ('map_w_mm',          1880.0),
            ('map_h_mm',          1410.0),
            ('img_w_px',          3840.0),
            ('img_h_px',          2160.0),
            ('flip_y',            False),
            ('control_hz',        20.0),
        ])

        g = self.get_parameter
        self.robot_id        = int(g('robot_id').value)
        self.route_id        = int(g('route_id').value)
        self.route_map_path  = g('route_map_path').value
        self.lookahead_px    = float(g('lookahead_px').value)
        self.base_speed      = float(g('base_speed_mps').value)
        self.max_angular     = float(g('max_angular_rad').value)
        self.goal_tol_px     = float(g('goal_tolerance_px').value)
        self.web_port        = int(g('web_port').value)
        self.map_w_mm        = float(g('map_w_mm').value)
        self.map_h_mm        = float(g('map_h_mm').value)
        self.img_w_px        = float(g('img_w_px').value)
        self.img_h_px        = float(g('img_h_px').value)
        self.flip_y          = bool(g('flip_y').value)
        self.control_hz      = float(g('control_hz').value)

        # 스케일 (mm → px)
        self.sx = self.img_w_px / self.map_w_mm
        self.sy = self.img_h_px / self.map_h_mm

        # 상태
        self._lock          = threading.Lock()
        self.enabled        = False
        self.status_text    = 'IDLE'
        self.latest_pose_px = None   # (x, y)
        self.latest_yaw     = 0.0
        self.latest_source  = -1
        self.latest_conf    = 0
        self.current_idx    = 0
        self.last_target    = None   # (x, y) for status

        # 경로 로드
        self.routes = {}
        self._load_route_map()
        self._set_route(self.route_id)

        # ROS2
        self._sub = self.create_subscription(
            RobotPose,
            f'/robot_{self.robot_id}/pose',
            self._pose_cb,
            10,
        )
        self._pub = self.create_publisher(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel_safe',
            10,
        )
        self.create_timer(1.0 / self.control_hz, self._control_loop)

        # 웹 UI 스레드
        self._start_web_server()

        self.get_logger().info(
            f'[robot_{self.robot_id}] WaypointFollower ready '
            f'(route={self.route_id}, lookahead={self.lookahead_px}px, '
            f'web=:{self.web_port})'
        )

    # ── 경로 로드 ─────────────────────────────────────────────────

    def _load_route_map(self):
        if not os.path.isfile(self.route_map_path):
            self.get_logger().error(
                f'route_map.json not found: {self.route_map_path}')
            self.routes = {}
            return
        with open(self.route_map_path, 'r') as f:
            data = json.load(f)
        for r in data.get('routes', []):
            rid = int(r['route_id'])
            wps = np.array(
                [[w['x'], w['y']] for w in r['waypoints']],
                dtype=np.float64,
            )
            self.routes[rid] = {
                'name':   r.get('route_name', f'route{rid}'),
                'closed': bool(r.get('closed', True)),
                'dir':    r.get('direction', '?'),
                'wps':    wps,
            }
        self.get_logger().info(
            f'Loaded {len(self.routes)} routes from {self.route_map_path}')

    def _set_route(self, rid):
        if rid not in self.routes:
            self.get_logger().warn(f'route_id {rid} not in map')
            return False
        with self._lock:
            self.route_id    = rid
            self.current_idx = 0
            self.status_text = 'IDLE'
        return True

    # ── Pose 콜백 ─────────────────────────────────────────────────

    def _pose_cb(self, msg: RobotPose):
        if msg.robot_id != self.robot_id:
            return
        x_px = msg.x_mm * self.sx
        y_px = msg.y_mm * self.sy
        if self.flip_y:
            y_px = self.img_h_px - y_px
        yaw = msg.theta_mrad / 1000.0
        with self._lock:
            self.latest_pose_px = (x_px, y_px)
            self.latest_yaw     = yaw
            self.latest_source  = int(msg.source)
            self.latest_conf    = int(msg.confidence_pct)

    # ── 제어 루프 ─────────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            enabled  = self.enabled
            pose     = self.latest_pose_px
            yaw      = self.latest_yaw
            rid      = self.route_id
            idx      = self.current_idx

        if not enabled or pose is None or rid not in self.routes:
            self._publish_zero()
            return

        route = self.routes[rid]
        wps   = route['wps']
        closed = route['closed']
        n = len(wps)
        if n == 0:
            self._publish_zero()
            return

        rx, ry = pose

        # 1) 최근접 waypoint (현재 idx 주변 윈도우 우선)
        idx = self._find_nearest(wps, rx, ry, idx, closed)

        # 2) open path 종료 체크
        if not closed:
            last = wps[-1]
            if math.hypot(last[0] - rx, last[1] - ry) < self.goal_tol_px:
                with self._lock:
                    self.enabled     = False
                    self.status_text = 'DONE'
                    self.current_idx = idx
                self._publish_zero()
                return

        # 3) lookahead waypoint 선정
        tgt_idx = self._find_lookahead(wps, idx, closed, self.lookahead_px)
        tx, ty  = wps[tgt_idx]

        # 4) Pure Pursuit
        dx = tx - rx
        dy = ty - ry
        ld = math.hypot(dx, dy)
        if ld < 1e-3:
            self._publish_zero()
            return

        target_angle = math.atan2(dy, dx)   # 이미지 y_down 좌표계
        alpha        = wrap_angle(target_angle - yaw)

        # curvature (단위: 1/px). angular_z = v * curvature
        # v 단위(m/s)와 ld 단위(px) 혼합이지만 튜닝으로 스케일 맞춤.
        # max_angular로 클램프하여 실용적으로 동작.
        curvature = 2.0 * math.sin(alpha) / ld
        w = self.base_speed * curvature * 100.0   # 스케일 게인 (튜닝값)

        w = max(-self.max_angular, min(self.max_angular, w))

        twist = Twist()
        twist.linear.x  = self.base_speed
        twist.angular.z = w
        self._pub.publish(twist)

        with self._lock:
            self.current_idx = idx
            self.last_target = (float(tx), float(ty))
            self.status_text = 'RUNNING'

    def _publish_zero(self):
        t = Twist()
        self._pub.publish(t)

    def _find_nearest(self, wps, rx, ry, hint_idx, closed):
        # 전체 검색 (2435 WP면 O(n) 충분)
        diffs = wps - np.array([rx, ry])
        d2 = (diffs ** 2).sum(axis=1)
        return int(np.argmin(d2))

    def _find_lookahead(self, wps, start_idx, closed, ld_px):
        n = len(wps)
        acc = 0.0
        i = start_idx
        while True:
            j = (i + 1) % n if closed else i + 1
            if not closed and j >= n:
                return n - 1
            seg = math.hypot(
                wps[j][0] - wps[i][0],
                wps[j][1] - wps[i][1],
            )
            acc += seg
            if acc >= ld_px:
                return j
            i = j
            if closed and i == start_idx:
                return j   # 한 바퀴 돌았다

    # ── 웹 서버 ───────────────────────────────────────────────────

    def _start_web_server(self):
        follower = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                self.do_GET()

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                query = self.path.split('?', 1)[1] if '?' in self.path else ''

                if path == '/':
                    html = follower._render_index().encode('utf-8')
                    self._send(200, 'text/html; charset=utf-8', html)
                    return
                if path == '/start':
                    with follower._lock:
                        follower.enabled = True
                        follower.status_text = 'RUNNING'
                    self._send(200, 'text/plain', b'OK')
                    return
                if path == '/stop':
                    with follower._lock:
                        follower.enabled = False
                        follower.status_text = 'IDLE'
                    follower._publish_zero()
                    self._send(200, 'text/plain', b'OK')
                    return
                if path == '/set_route':
                    rid = None
                    for kv in query.split('&'):
                        if kv.startswith('id='):
                            try:
                                rid = int(kv[3:])
                            except ValueError:
                                pass
                    ok = follower._set_route(rid) if rid else False
                    follower._publish_zero()
                    self._send(200, 'text/plain',
                               b'OK' if ok else b'BAD_ROUTE')
                    return
                if path == '/status':
                    body = json.dumps(
                        follower._status_dict()).encode('utf-8')
                    self._send(200, 'application/json', body)
                    return
                self._send(404, 'text/plain', b'not found')

        port = self.web_port
        server = HTTPServer(('0.0.0.0', port), Handler)

        def _run():
            server.serve_forever()

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        self._http_server = server

    def _status_dict(self):
        with self._lock:
            route = self.routes.get(self.route_id)
            n = len(route['wps']) if route else 0
            return {
                'enabled':   self.enabled,
                'status':    self.status_text,
                'route_id':  self.route_id,
                'route_name': route['name'] if route else '',
                'direction': route['dir'] if route else '',
                'closed':    route['closed'] if route else False,
                'total_wp':  n,
                'wp_idx':    self.current_idx,
                'pose_px':   self.latest_pose_px,
                'yaw_deg':   round(math.degrees(self.latest_yaw), 1),
                'target_px': self.last_target,
                'source':    self.latest_source,
                'conf':      self.latest_conf,
                'robot_id':  self.robot_id,
                'sx':        round(self.sx, 4),
                'sy':        round(self.sy, 4),
                'flip_y':    self.flip_y,
            }

    def _render_index(self):
        options = '\n'.join(
            f'<option value="{rid}">{rid}: {r["name"]} ({r["dir"]}, '
            f'{len(r["wps"])}wp)</option>'
            for rid, r in sorted(self.routes.items())
        )
        return f'''<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Waypoint Follower</title>
<style>
body{{background:#111;color:#fff;font-family:sans-serif;margin:20px}}
h1{{margin:0 0 8px}}
.row{{display:flex;gap:10px;margin:10px 0}}
button{{padding:14px 28px;font-size:20px;color:#fff;border:none;
       border-radius:8px;cursor:pointer;font-weight:bold}}
.start{{background:#2ecc40}} .stop{{background:#ff4136}}
select{{font-size:18px;padding:8px;border-radius:6px}}
pre{{background:#222;padding:12px;border-radius:8px;font-size:14px;
     min-width:520px;white-space:pre-wrap}}
.kv{{font-family:monospace;font-size:16px}}
#status_box{{font-size:18px;margin:6px 0;font-weight:bold}}
</style></head><body>
<h1>Waypoint Follower — robot_{self.robot_id}</h1>
<div id="status_box">state: loading...</div>
<div class="row">
  <label class="kv">Route:</label>
  <select id="sel_route" onchange="setRoute(this.value)">
    {options}
  </select>
</div>
<div class="row">
  <button class="start" onclick="cmd('start')">START</button>
  <button class="stop"  onclick="cmd('stop')">STOP</button>
</div>
<pre id="status_json">loading...</pre>
<script>
function cmd(c){{ fetch('/'+c, {{method:'POST'}}).then(refresh); }}
function setRoute(id){{
  fetch('/set_route?id='+id, {{method:'POST'}}).then(refresh);
}}
function refresh(){{
  fetch('/status').then(r=>r.json()).then(s=>{{
    document.getElementById('status_box').innerText =
      'state: ' + s.status + '  |  route=' + s.route_id +
      ' (' + s.route_name + ')  idx=' + s.wp_idx + '/' + s.total_wp;
    document.getElementById('status_json').innerText =
      JSON.stringify(s, null, 2);
    document.getElementById('sel_route').value = s.route_id;
  }});
}}
setInterval(refresh, 500);
refresh();
</script>
</body></html>'''


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
