#!/usr/bin/env python3
"""
Waypoint Follower Node (ARASEO-DALIMI)

Host PC가 발행하는 핑키 위치(/robot_{id}/pose, mm)를 구독하고,
map_vectors_mm.json의 세그먼트(mm)를 Pure Pursuit로 추종해서
/robot_{id}/cmd_vel_safe (Twist)를 발행한다.

웹 UI (포트 8082)에서 세그먼트 선택 + START/STOP 제어.

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


DEFAULT_ROUTE_MAP = '/home/pinky/araseo_dalimi_ws/map_vectors_mm.json'


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameters('', [
            ('robot_id',          14),
            ('route_id',          1),
            ('route_map_path',    DEFAULT_ROUTE_MAP),
            ('lookahead_mm',      200.0),
            ('base_speed_mps',    0.12),
            ('max_angular_rad',   1.5),
            ('goal_tolerance_mm', 20.0),
            ('web_port',          8082),
            ('map_w_mm',          1880.0),
            ('map_h_mm',          1410.0),
            ('flip_y',            True),   # map_vectors_mm.json = y-up, GPS = y-down → 반전 필요
            ('control_hz',        20.0),
        ])

        g = self.get_parameter
        self.robot_id        = int(g('robot_id').value)
        self.route_id        = int(g('route_id').value)
        self.route_map_path  = g('route_map_path').value
        self.lookahead_mm    = float(g('lookahead_mm').value)
        self.base_speed      = float(g('base_speed_mps').value)
        self.max_angular     = float(g('max_angular_rad').value)
        self.goal_tol_mm     = float(g('goal_tolerance_mm').value)
        self.web_port        = int(g('web_port').value)
        self.map_w_mm        = float(g('map_w_mm').value)
        self.map_h_mm        = float(g('map_h_mm').value)
        self.flip_y          = bool(g('flip_y').value)
        self.control_hz      = float(g('control_hz').value)

        # 상태
        self._lock          = threading.Lock()
        self.enabled        = False
        self.status_text    = 'IDLE'
        self.latest_pose_mm = None   # (x, y) in mm
        self.latest_yaw     = 0.0
        self.latest_source  = -1
        self.latest_conf    = 0
        self.current_idx    = 0
        self.last_target    = None   # (x, y) in mm for status
        self._pose_received = False  # 첫 pose 수신 여부

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
            f'(route={self.route_id}, lookahead={self.lookahead_mm}mm, '
            f'web=:{self.web_port})'
        )

    # ── 경로 로드 ─────────────────────────────────────────────────

    def _load_route_map(self):
        if not os.path.isfile(self.route_map_path):
            self.get_logger().error(
                f'map_vectors_mm.json not found: {self.route_map_path}')
            self.routes = {}
            return
        with open(self.route_map_path, 'r') as f:
            data = json.load(f)
        # segments 키(map_vectors_mm.json) 또는 routes 키(route_map_mm.json) 모두 지원
        items = data.get('segments', data.get('routes', []))
        for r in items:
            rid = int(r.get('segment_id', r.get('route_id', 0)))
            wps = np.array(
                [[w['x'], w['y']] for w in r['waypoints']],
                dtype=np.float64,
            )
            self.routes[rid] = {
                'name':   r.get('name', f'route{rid}'),
                'closed': bool(r.get('is_closed', False)),
                'dir':    r.get('direction', ''),
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
        x_mm    = float(msg.x_mm)
        yaw_raw = msg.theta_mrad / 1000.0
        if self.flip_y:
            # GPS는 y-down(이미지 상단 기준), map_vectors는 y-up(하단 기준)
            # y 반전 시 yaw 방향도 부호 반전 (atan2(dy) → atan2(-dy) = -angle)
            y_mm = self.map_h_mm - float(msg.y_mm)
            yaw  = -yaw_raw
        else:
            y_mm = float(msg.y_mm)
            yaw  = yaw_raw
        with self._lock:
            self.latest_pose_mm = (x_mm, y_mm)
            self.latest_yaw     = yaw
            self.latest_source  = int(msg.source)
            self.latest_conf    = int(msg.confidence_pct)
            first = not self._pose_received
            self._pose_received = True
        if first:
            self.get_logger().info(
                f'[robot_{self.robot_id}] 첫 pose 수신: '
                f'({x_mm:.0f}, {y_mm:.0f}) mm, '
                f'yaw={math.degrees(yaw):.1f}°, '
                f'source={int(msg.source)}'
            )

    # ── 제어 루프 ─────────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            enabled  = self.enabled
            pose     = self.latest_pose_mm
            yaw      = self.latest_yaw
            rid      = self.route_id
            idx      = self.current_idx

        if not enabled or pose is None or rid not in self.routes:
            self._publish_zero()
            # pose가 없으면 경고 (10초마다)
            if enabled and pose is None:
                self.get_logger().warn(
                    f'[robot_{self.robot_id}] pose 미수신 — '
                    f'/robot_{self.robot_id}/pose 토픽 확인 필요',
                    throttle_duration_sec=10.0,
                )
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
            if math.hypot(last[0] - rx, last[1] - ry) < self.goal_tol_mm:
                with self._lock:
                    self.enabled     = False
                    self.status_text = 'DONE'
                    self.current_idx = idx
                self._publish_zero()
                return

        # 3) lookahead waypoint 선정
        tgt_idx = self._find_lookahead(wps, idx, closed, self.lookahead_mm)
        tx, ty  = wps[tgt_idx]

        # 4) Pure Pursuit
        dx = tx - rx
        dy = ty - ry
        ld = math.hypot(dx, dy)
        if ld < 1e-3:
            self._publish_zero()
            return

        target_angle = math.atan2(dy, dx)   # y-up (map_vectors 좌표계, 수학 표준)
        alpha        = wrap_angle(target_angle - yaw)

        # Pure Pursuit: ld(mm) → ld(m) 변환 후 표준 공식 적용
        # curvature(1/m) = 2*sin(alpha) / ld_m
        # angular_z(rad/s) = v(m/s) * curvature(1/m)
        ld_m      = ld / 1000.0
        curvature = 2.0 * math.sin(alpha) / ld_m
        w         = self.base_speed * curvature

        w = max(-self.max_angular, min(self.max_angular, w))

        twist = Twist()
        twist.linear.x  = self.base_speed
        twist.angular.z = w
        self._pub.publish(twist)
        self._pub_count = getattr(self, '_pub_count', 0) + 1
        if self._pub_count <= 5 or self._pub_count % 100 == 0:
            self.get_logger().info(
                f'[publish#{self._pub_count}] linear={twist.linear.x:.3f} '
                f'angular={twist.angular.z:.4f} pose=({rx:.0f},{ry:.0f}) '
                f'target=({tx:.0f},{ty:.0f})')

        with self._lock:
            self.current_idx = idx
            self.last_target = (float(tx), float(ty))
            self.status_text = 'RUNNING'

    def _publish_zero(self):
        t = Twist()
        self._pub.publish(t)

    def _find_nearest(self, wps, rx, ry, hint_idx, closed):
        # hint_idx 기준 앞쪽 방향 우선 탐색 (역행 방지)
        # 뒤 25 / 앞 75 비율의 윈도우 내에서만 탐색
        # n < window 이면 전체 탐색으로 fallback (세그먼트가 짧은 경우)
        n = len(wps)
        window = 150
        if n <= window:
            # 세그먼트가 짧아 윈도우가 전체를 커버 → 역행 위험 없음, 전체 탐색
            diffs = wps - np.array([rx, ry])
            d2 = (diffs ** 2).sum(axis=1)
            return int(np.argmin(d2))
        back = window // 4    # 37
        fwd  = window - back  # 113
        if closed:
            indices = [(hint_idx + i) % n for i in range(-back, fwd)]
        else:
            lo = max(0, hint_idx - back)
            hi = min(n, hint_idx + fwd)
            indices = list(range(lo, hi))
        idx_arr = np.array(indices)
        diffs = wps[idx_arr] - np.array([rx, ry])
        d2 = (diffs ** 2).sum(axis=1)
        best = int(np.argmin(d2))
        return indices[best]

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
                if path == '/map':
                    html = follower._render_map().encode('utf-8')
                    self._send(200, 'text/html; charset=utf-8', html)
                    return
                if path == '/map_data':
                    body = json.dumps(
                        follower._map_data_dict()).encode('utf-8')
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
                'pose_mm':   self.latest_pose_mm,
                'yaw_deg':   round(math.degrees(self.latest_yaw), 1),
                'target_mm': self.last_target,
                'source':    self.latest_source,
                'conf':      self.latest_conf,
                'robot_id':  self.robot_id,
                'flip_y':    self.flip_y,
            }

    def _map_data_dict(self):
        """모든 경로 waypoint를 mm 좌표로 반환 (정적 데이터)."""
        routes_out = []
        for rid, r in sorted(self.routes.items()):
            wps_mm = [
                {'x': round(float(w[0]), 1),
                 'y': round(float(w[1]), 1)}
                for w in r['wps']
            ]
            routes_out.append({
                'route_id': rid,
                'name':     r['name'],
                'dir':      r['dir'],
                'closed':   r['closed'],
                'wps':      wps_mm,
            })
        return {
            'map_w_mm': self.map_w_mm,
            'map_h_mm': self.map_h_mm,
            'routes':   routes_out,
        }

    def _render_map(self):
        ROUTE_COLORS = [
            '#e74c3c','#e67e22','#f1c40f','#2ecc71','#1abc9c',
            '#3498db','#9b59b6','#e91e63','#00bcd4','#8bc34a',
            '#ff5722','#607d8b','#795548','#03a9f4',
        ]
        return f'''<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Map — robot_{self.robot_id}</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{background:#0f0f1a;color:#fff;font-family:sans-serif;
     display:flex;flex-direction:column;height:100vh;overflow:hidden}}
#topbar{{display:flex;align-items:center;gap:16px;padding:8px 14px;
         background:#1a1a2e;border-bottom:1px solid #333;flex-shrink:0}}
#topbar h2{{font-size:15px;white-space:nowrap}}
#info{{font-size:13px;color:#aaa;flex:1}}
#conf_badge{{padding:3px 10px;border-radius:12px;font-size:13px;
             font-weight:bold;background:#333}}
#map_wrap{{flex:1;overflow:hidden;position:relative}}
svg{{width:100%;height:100%;display:block}}
.route-path{{fill:none;stroke-width:2;opacity:0.75}}
#robot_group circle{{filter:drop-shadow(0 0 6px #00ff88)}}
#target_line{{stroke:#ff6b6b;stroke-width:1.5;stroke-dasharray:6,4;opacity:0.8}}
#legend{{position:absolute;top:8px;right:8px;background:#1a1a2ecc;
         border:1px solid #333;border-radius:8px;padding:8px 12px;
         font-size:11px;max-height:calc(100vh - 60px);overflow-y:auto}}
.leg-row{{display:flex;align-items:center;gap:6px;margin:2px 0}}
.leg-dot{{width:10px;height:10px;border-radius:50%;flex-shrink:0}}
</style></head><body>
<div id="topbar">
  <h2>Robot {self.robot_id} — Live Map</h2>
  <div id="info">위치: -- | yaw: -- | source: -- </div>
  <span id="conf_badge">conf: --</span>
  <a href="/" style="color:#aaa;font-size:13px;text-decoration:none">[ 제어 ]</a>
</div>
<div id="map_wrap">
  <svg id="svg_map" viewBox="0 0 {self.map_w_mm} {self.map_h_mm}"
       preserveAspectRatio="xMidYMid meet">
    <rect width="{self.map_w_mm}" height="{self.map_h_mm}"
          fill="#111122" rx="4"/>
    <g id="routes_layer"></g>
    <line id="target_line" x1="0" y1="0" x2="0" y2="0"/>
    <g id="robot_group">
      <circle id="robot_dot" cx="-999" cy="-999" r="18"
              fill="#00ff88" stroke="white" stroke-width="2"/>
      <line id="robot_arrow" x1="0" y1="0" x2="0" y2="0"
            stroke="#00ff88" stroke-width="4" stroke-linecap="round"/>
    </g>
  </svg>
  <div id="legend"></div>
</div>
<script>
const COLORS={json.dumps(ROUTE_COLORS)};
const MAP_W={self.map_w_mm}, MAP_H={self.map_h_mm};
let mapLoaded=false;

function svgEl(tag,attrs){{
  const el=document.createElementNS('http://www.w3.org/2000/svg',tag);
  for(const[k,v] of Object.entries(attrs)) el.setAttribute(k,v);
  return el;
}}

async function loadMap(){{
  const d=await fetch('/map_data').then(r=>r.json());
  const layer=document.getElementById('routes_layer');
  const leg=document.getElementById('legend');
  d.routes.forEach((r,i)=>{{
    const c=COLORS[i%COLORS.length];
    const pts=r.wps.map(w=>w.x+','+w.y).join(' ');
    const closed=r.closed?' '+r.wps[0].x+','+r.wps[0].y:'';
    const path=svgEl('polyline',{{
      points:pts+closed,
      class:'route-path',
      stroke:c,
      'data-rid':r.route_id
    }});
    layer.appendChild(path);
    // legend
    const row=document.createElement('div');
    row.className='leg-row';
    row.innerHTML=`<div class="leg-dot" style="background:${{c}}"></div>
      <span>R${{r.route_id}} ${{r.name}} (${{r.dir}})</span>`;
    leg.appendChild(row);
  }});
  mapLoaded=true;
}}

function refresh(){{
  fetch('/status').then(r=>r.json()).then(s=>{{
    const src=s.source===0?'GPS':'Odom';
    const conf=s.conf;
    const px=s.pose_mm;
    const dot=document.getElementById('robot_dot');
    const arrow=document.getElementById('robot_arrow');
    const tline=document.getElementById('target_line');

    if(px && (px[0]||px[1])){{
      const xm=px[0], ym=px[1];
      const yaw=s.yaw_deg*Math.PI/180;
      const alen=50;
      const ax2=xm+alen*Math.cos(yaw), ay2=ym+alen*Math.sin(yaw);
      dot.setAttribute('cx',xm); dot.setAttribute('cy',ym);
      arrow.setAttribute('x1',xm); arrow.setAttribute('y1',ym);
      arrow.setAttribute('x2',ax2); arrow.setAttribute('y2',ay2);
      dot.setAttribute('fill', conf<50?'#ff6b6b':conf<80?'#f39c12':'#00ff88');

      if(s.target_mm){{
        const tx=s.target_mm[0], ty=s.target_mm[1];
        tline.setAttribute('x1',xm); tline.setAttribute('y1',ym);
        tline.setAttribute('x2',tx); tline.setAttribute('y2',ty);
      }} else {{
        tline.setAttribute('x1',0); tline.setAttribute('y1',0);
        tline.setAttribute('x2',0); tline.setAttribute('y2',0);
      }}

      document.getElementById('info').textContent=
        `위치: (${{xm.toFixed(0)}}, ${{ym.toFixed(0)}}) mm | `+
        `yaw: ${{s.yaw_deg}}° | route: R${{s.route_id}} ${{s.route_name}} | `+
        `wp: ${{s.wp_idx}}/${{s.total_wp}} | src: ${{src}}`;
    }}

    const badge=document.getElementById('conf_badge');
    badge.textContent='conf: '+conf+'%';
    badge.style.background=conf<50?'#c0392b':conf<80?'#d35400':'#27ae60';
  }});
}}

loadMap().then(()=>{{
  setInterval(refresh,500);
  refresh();
}});
</script>
</body></html>'''

    def _render_index(self):
        options = '\n'.join(
            f'<option value="{rid}">{rid}: {r["name"]} '
            f'({len(r["wps"])}wp)</option>'
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
