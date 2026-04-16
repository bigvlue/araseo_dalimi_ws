#!/usr/bin/env python3
"""
Map Viewer Node (ARASEO-DALIMI)

map_vectors_mm.json 벡터맵 위에 로봇의 실시간 위치/방향을 표시한다.
waypoint_follower 없이 독립 실행 가능.

웹 브라우저: http://<pi-ip>:8083/

실행 전: export ROS_DOMAIN_ID=204
"""
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node

from araseo_dalimi_interfaces.msg import RobotPose


DEFAULT_ROUTE_MAP = '/home/pinky/araseo_dalimi_ws/map_vectors_mm.json'

ROUTE_COLORS = [
    '#e74c3c', '#e67e22', '#f1c40f', '#2ecc71', '#1abc9c',
    '#3498db', '#9b59b6', '#e91e63', '#00bcd4', '#8bc34a',
    '#ff5722', '#607d8b', '#795548', '#03a9f4',
]


class MapViewerNode(Node):

    def __init__(self):
        super().__init__('map_viewer')

        self.declare_parameters('', [
            ('robot_id',       14),
            ('route_map_path', DEFAULT_ROUTE_MAP),
            ('web_port',       8083),
            ('map_w_mm',       1880.0),
            ('map_h_mm',       1410.0),
            ('flip_y',         False),
        ])

        g = self.get_parameter
        self.robot_id       = int(g('robot_id').value)
        self.route_map_path = g('route_map_path').value
        self.web_port       = int(g('web_port').value)
        self.map_w_mm       = float(g('map_w_mm').value)
        self.map_h_mm       = float(g('map_h_mm').value)
        self.flip_y         = bool(g('flip_y').value)

        self._lock       = threading.Lock()
        self.pose_mm     = None   # (x, y)
        self.yaw_rad     = 0.0
        self.source      = -1
        self.conf        = 0

        # 경로 데이터 로드
        self.routes = self._load_routes()

        # ROS2 구독
        self.create_subscription(
            RobotPose,
            f'/robot_{self.robot_id}/pose',
            self._pose_cb,
            10,
        )

        self._start_web_server()

        self.get_logger().info(
            f'[robot_{self.robot_id}] MapViewer 시작 — '
            f'http://0.0.0.0:{self.web_port}/'
        )

    # ── 경로 로드 ──────────────────────────────────────────────────

    def _load_routes(self):
        if not os.path.isfile(self.route_map_path):
            self.get_logger().error(f'map_vectors_mm.json not found: {self.route_map_path}')
            return []
        with open(self.route_map_path, 'r') as f:
            data = json.load(f)
        routes = []
        for r in data.get('segments', []):
            sid = int(r['segment_id'])
            routes.append({
                'route_id': sid,
                'name':     r.get('name', f'seg{sid}'),
                'dir':      '',
                'closed':   bool(r.get('is_closed', False)),
                'wps':      [[w['x'], w['y']] for w in r['waypoints']],
            })
        self.get_logger().info(f'{len(routes)}개 세그먼트 로드 완료')
        return routes

    # ── Pose 콜백 ──────────────────────────────────────────────────

    def _pose_cb(self, msg: RobotPose):
        if msg.robot_id != self.robot_id:
            return
        x = float(msg.x_mm)
        # GPS y-down을 y-up으로 변환 (map_vectors_mm.json 좌표계)
        y = self.map_h_mm - float(msg.y_mm)
        with self._lock:
            self.pose_mm = (x, y)
            # GPS yaw는 y-down 기준 → y-up 기준으로 부호 반전
            self.yaw_rad = -(msg.theta_mrad / 1000.0)
            self.source  = int(msg.source)
            self.conf    = int(msg.confidence_pct)

    # ── 웹 서버 ───────────────────────────────────────────────────

    def _start_web_server(self):
        viewer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split('?', 1)[0]

                if path == '/':
                    html = viewer._render_page().encode('utf-8')
                    self._send(200, 'text/html; charset=utf-8', html)
                elif path == '/map_data':
                    body = json.dumps(viewer._map_data()).encode('utf-8')
                    self._send(200, 'application/json', body)
                elif path == '/pose':
                    body = json.dumps(viewer._pose_data()).encode('utf-8')
                    self._send(200, 'application/json', body)
                else:
                    self._send(404, 'text/plain', b'not found')

        server = HTTPServer(('0.0.0.0', self.web_port), Handler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self._http_server = server

    def _map_data(self):
        return {
            'map_w_mm': self.map_w_mm,
            'map_h_mm': self.map_h_mm,
            'routes': [
                {
                    'route_id': r['route_id'],
                    'name':     r['name'],
                    'dir':      r['dir'],
                    'closed':   r['closed'],
                    'wps':      r['wps'],
                }
                for r in self.routes
            ],
        }

    def _pose_data(self):
        with self._lock:
            return {
                'pose_mm': self.pose_mm,
                'yaw_deg': round(math.degrees(self.yaw_rad), 1),
                'source':  self.source,
                'conf':    self.conf,
            }

    # ── SVG / HTML 헬퍼 ───────────────────────────────────────────

    def _svg_routes(self):
        """경로 polyline + 시작점 마커를 SVG 문자열로 반환.
        map_vectors_mm.json은 y-up(하단 기준), SVG는 y-down(상단 기준)이므로
        y_svg = H - y_map 변환 적용. x는 변환 없음."""
        H = self.map_h_mm

        def fx(x):
            return x

        def fy(y):
            return H - y   # y-up → SVG y-down

        parts = []
        for i, r in enumerate(self.routes):
            color = ROUTE_COLORS[i % len(ROUTE_COLORS)]
            pts = ' '.join(f"{fx(w[0])},{fy(w[1])}" for w in r['wps'])
            if r['closed'] and r['wps']:
                pts += f" {fx(r['wps'][0][0])},{fy(r['wps'][0][1])}"
            parts.append(
                f'<polyline points="{pts}" fill="none" stroke="{color}" '
                f'stroke-width="3" opacity="0.75" '
                f'stroke-linecap="round" stroke-linejoin="round"/>'
            )
            if r['wps']:
                sx = fx(r['wps'][0][0])
                sy = fy(r['wps'][0][1])
                parts.append(
                    f'<circle cx="{sx}" cy="{sy}" r="8" '
                    f'fill="{color}" opacity="0.9"/>'
                )
        return '\n    '.join(parts)

    def _legend_html(self):
        """범례 HTML 문자열 반환."""
        rows = []
        for i, r in enumerate(self.routes):
            color = ROUTE_COLORS[i % len(ROUTE_COLORS)]
            label = f'S{r["route_id"]} {r["name"]}'
            rows.append(
                f'<div style="display:flex;align-items:center;gap:6px;margin:3px 0">'
                f'<div style="width:10px;height:10px;border-radius:50%;'
                f'background:{color};flex-shrink:0"></div>'
                f'<span>{label}</span></div>'
            )
        return '\n'.join(rows)

    def _grid_lines(self):
        """200mm 간격 격자선 SVG 문자열 반환."""
        lines = []
        step = 200
        x = step
        while x < self.map_w_mm:
            lines.append(f'<line x1="{x}" y1="0" x2="{x}" y2="{self.map_h_mm}"/>')
            x += step
        y = step
        while y < self.map_h_mm:
            lines.append(f'<line x1="0" y1="{y}" x2="{self.map_w_mm}" y2="{y}"/>')
            y += step
        return '\n    '.join(lines)

    # ── HTML 렌더 ─────────────────────────────────────────────────

    def _render_page(self):
        W = self.map_w_mm
        H = self.map_h_mm
        svg_routes  = self._svg_routes()
        legend_html = self._legend_html()
        grid_lines  = self._grid_lines()

        return f'''<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Map Viewer — robot_{self.robot_id}</title>
<style>
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: #0d0d1a; color: #fff; font-family: sans-serif; }}
#topbar {{
  position: fixed; top: 0; left: 0; right: 0; height: 40px; z-index: 10;
  display: flex; align-items: center; gap: 12px; padding: 0 14px;
  background: #141428; border-bottom: 1px solid #2a2a4a;
}}
#topbar h2 {{ font-size: 14px; white-space: nowrap; color: #a0c4ff; }}
#info {{ font-size: 12px; color: #888; flex: 1; }}
.badge {{
  padding: 2px 10px; border-radius: 10px;
  font-size: 12px; font-weight: bold; background: #333; white-space: nowrap;
}}
#map_wrap {{
  position: fixed; top: 40px; left: 0; right: 0; bottom: 0;
  background: #10101e;
}}
#svg_map {{ display: block; width: 100%; height: 100%; }}
#legend {{
  position: absolute; top: 8px; right: 8px;
  background: rgba(20,20,40,0.92); border: 1px solid #2a2a4a;
  border-radius: 8px; padding: 7px 11px; font-size: 11px;
  max-height: calc(100% - 20px); overflow-y: auto; white-space: nowrap;
}}
</style>
</head><body>

<div id="topbar">
  <h2>robot_{self.robot_id} — Map Viewer</h2>
  <div id="info">위치 수신 대기중...</div>
  <span class="badge" id="src_badge">--</span>
  <span class="badge" id="conf_badge">conf: --</span>
</div>

<div id="map_wrap">
  <svg id="svg_map"
       viewBox="0 0 {W} {H}"
       preserveAspectRatio="xMidYMid meet"
       xmlns="http://www.w3.org/2000/svg">

    <rect width="{W}" height="{H}" fill="#10101e"/>

    <g stroke="#1e1e36" stroke-width="1">
    {grid_lines}
    </g>

    {svg_routes}

    <g id="robot_grp" visibility="hidden">
      <line id="robot_hdg" x1="0" y1="0" x2="0" y2="0"
            stroke="#00ff88" stroke-width="6" stroke-linecap="round"/>
      <circle id="robot_dot" cx="-999" cy="-999" r="18"
              fill="#00ff88" stroke="#fff" stroke-width="3"/>
    </g>
  </svg>

  <div id="legend">{legend_html}</div>
</div>

<script>
const ARROW = 65;
const MAP_H = {H};  // y-up → SVG y-down 변환용

function refresh() {{
  fetch('/pose').then(r => r.json()).then(s => {{
    const pm  = s.pose_mm;
    const grp = document.getElementById('robot_grp');
    const dot = document.getElementById('robot_dot');
    const hdg = document.getElementById('robot_hdg');
    const inf = document.getElementById('info');
    const cbg = document.getElementById('conf_badge');
    const sbg = document.getElementById('src_badge');

    if (!pm) {{ grp.setAttribute('visibility','hidden'); return; }}

    const x = pm[0];
    const y_up = pm[1];          // y-up (map_vectors 좌표계)
    const cy = MAP_H - y_up;     // SVG y-down 변환
    const yaw = s.yaw_deg * Math.PI / 180;
    const clr = s.conf < 50 ? '#ff4757' : s.conf < 80 ? '#ffa502' : '#00ff88';

    grp.setAttribute('visibility','visible');
    dot.setAttribute('cx', x); dot.setAttribute('cy', cy);
    dot.setAttribute('fill', clr); dot.setAttribute('stroke', '#fff');
    hdg.setAttribute('x1', x); hdg.setAttribute('y1', cy);
    hdg.setAttribute('x2', x + ARROW*Math.cos(yaw));
    hdg.setAttribute('y2', cy - ARROW*Math.sin(yaw));  // y-up 방향이므로 SVG에서 빼기
    hdg.setAttribute('stroke', clr);

    inf.textContent = '위치: (' + x.toFixed(0) + ', ' + y_up.toFixed(0) + ') mm  |  방향: ' + s.yaw_deg.toFixed(1) + '°';

    cbg.textContent = 'conf: ' + s.conf + '%';
    cbg.style.background = s.conf < 50 ? '#5c1a1a' : s.conf < 80 ? '#5c3a00' : '#1a4a2a';

    const isgps = s.source === 0;
    sbg.textContent = isgps ? 'GPS-CAM' : 'ODOM';
    sbg.style.background = isgps ? '#1a3a4a' : '#3a2a1a';
    sbg.style.color       = isgps ? '#4fc3f7' : '#ffb74d';
  }}).catch(() => {{}});
}}

setInterval(refresh, 100);
refresh();
</script>
</body></html>'''


def main(args=None):
    rclpy.init(args=args)
    node = MapViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
