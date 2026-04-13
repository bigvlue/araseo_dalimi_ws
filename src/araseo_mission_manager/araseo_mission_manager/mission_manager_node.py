#!/usr/bin/env python3
"""
ARASEO-DALIMI Mission Manager Node
- SetMission 서비스로 다중 목적지 수신
- 토폴로지 A* 최단경로 계획
- Nav2 NavigateToPose 액션 클라이언트로 순차 실행
- MissionStatus 발행 (1Hz, 저빈도)
"""
import math
import heapq
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from araseo_dalimi_interfaces.msg import MissionStatus
from araseo_dalimi_interfaces.srv import SetMission, GetMissionStatus


# ── 토폴로지 그래프 ────────────────────────────────────────────────

class TopologyGraph:
    """미니 시티 도로 토폴로지 그래프 및 A* 경로 탐색"""

    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self._nodes = {}   # id → (x_mm, y_mm)
        self._edges = {}   # id → [(neighbor_id, cost), ...]

        for n in data['nodes']:
            self._nodes[n['id']] = (n['x'], n['y'])
            self._edges[n['id']] = []

        for e in data['edges']:
            a, b, cost = e[0], e[1], e[2]
            self._edges[a].append((b, cost))
            self._edges[b].append((a, cost))  # 양방향

    def nearest_node(self, x_mm: float, y_mm: float) -> str:
        """주어진 좌표에서 가장 가까운 노드 ID 반환"""
        best_id   = None
        best_dist = float('inf')
        for nid, (nx, ny) in self._nodes.items():
            d = math.sqrt((x_mm - nx)**2 + (y_mm - ny)**2)
            if d < best_dist:
                best_dist = d
                best_id   = nid
        return best_id

    def astar(self, start_id: str, goal_id: str) -> list:
        """
        A* 최단경로 탐색
        반환: 노드 ID 리스트 (start 포함, goal 포함)
        """
        if start_id == goal_id:
            return [start_id]

        def heuristic(a, b):
            ax, ay = self._nodes[a]
            bx, by = self._nodes[b]
            return math.sqrt((ax - bx)**2 + (ay - by)**2)

        open_set = [(0.0, start_id)]
        came_from = {}
        g_score = {start_id: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_id:
                # 경로 재구성
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_id)
                return list(reversed(path))

            for neighbor, cost in self._edges.get(current, []):
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    f = tentative_g + heuristic(neighbor, goal_id)
                    heapq.heappush(open_set, (f, neighbor))

        return []  # 경로 없음

    def node_pose(self, node_id: str) -> tuple:
        """노드 좌표 반환 (x_mm, y_mm)"""
        return self._nodes.get(node_id, (0, 0))


# ── 미션 관리 노드 ─────────────────────────────────────────────────

class MissionManagerNode(Node):

    STATE = MissionStatus

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameters('', [
            ('robot_id',              0),
            ('publish_hz',            1.0),
            ('nav2_action_timeout',   60.0),
            ('arrival_tolerance_m',   0.05),
            ('use_nav2',              True),
            ('loop_back_on_complete', False),
        ])

        g = self.get_parameter
        self.robot_id        = g('robot_id').value
        self.publish_hz      = g('publish_hz').value
        self.nav2_timeout    = g('nav2_action_timeout').value
        self.arrival_tol     = g('arrival_tolerance_m').value
        self.use_nav2        = g('use_nav2').value
        self.loop_mission    = g('loop_back_on_complete').value

        # 토폴로지 그래프 로드
        pkg_share = get_package_share_directory('araseo_mission_manager')
        topo_path = os.path.join(pkg_share, 'config', 'map_topology.yaml')
        self._graph = TopologyGraph(topo_path)
        self.get_logger().info(f'토폴로지 그래프 로드 완료: {topo_path}')

        # 미션 상태
        self._mission_id      = ''
        self._goals           = []        # [(x_m, y_m, theta_rad), ...]
        self._waypoints       = []        # 전체 웨이포인트 (A* 확장)
        self._current_wp_idx  = 0
        self._mission_state   = MissionStatus.IDLE
        self._is_navigating   = False
        self._nav_once_timer  = None

        # Nav2 액션 클라이언트
        self._cb_group = ReentrantCallbackGroup()
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            f'/robot_{self.robot_id}/navigate_to_pose',
            callback_group=self._cb_group
        )

        # 현재 로봇 위치 캐시
        self._current_x_m = 0.0
        self._current_y_m = 0.0

        # 서비스 서버
        self._set_mission_srv = self.create_service(
            SetMission,
            f'/robot_{self.robot_id}/set_mission',
            self._set_mission_callback,
            callback_group=self._cb_group
        )
        self._get_status_srv = self.create_service(
            GetMissionStatus,
            f'/robot_{self.robot_id}/get_mission_status',
            self._get_status_callback
        )

        # 퍼블리셔 (1Hz 저빈도)
        self._status_pub = self.create_publisher(
            MissionStatus,
            f'/robot_{self.robot_id}/mission_status',
            10
        )
        period = 1.0 / self.publish_hz
        self.create_timer(period, self._publish_status)

        self.get_logger().info(
            f'[robot_{self.robot_id}] MissionManager 시작 '
            f'(Nav2={self.use_nav2})'
        )

    # ── SetMission 서비스 ─────────────────────────────────────────

    def _set_mission_callback(self, request: SetMission.Request,
                               response: SetMission.Response):
        if self._mission_state == MissionStatus.RUNNING:
            response.success = False
            response.message = '미션 실행 중. 먼저 중단하세요.'
            return response

        if not request.goal_x_m:
            response.success = False
            response.message = '목적지가 비어있습니다.'
            return response

        # 목적지 저장
        self._goals = list(zip(
            request.goal_x_m,
            request.goal_y_m,
            request.goal_theta_rad
        ))
        self._mission_id    = request.mission_id or 'mission_default'
        self.loop_mission   = request.loop_mission
        self._current_wp_idx = 0

        # A* 경로 계획
        self._waypoints = self._plan_path()

        if not self._waypoints:
            response.success = False
            response.message = '경로 계획 실패'
            return response

        # 미션 시작
        self._mission_state = MissionStatus.RUNNING
        self.get_logger().info(
            f'[robot_{self.robot_id}] 미션 시작: {self._mission_id} '
            f'({len(self._goals)}개 목적지, '
            f'{len(self._waypoints)}개 웨이포인트)'
        )

        # 첫 웨이포인트로 이동 시작
        self._navigate_next()

        response.success            = True
        response.message            = f'미션 수락: {len(self._waypoints)}개 웨이포인트'
        response.accepted_mission_id = self._mission_id
        return response

    # ── GetMissionStatus 서비스 ───────────────────────────────────

    def _get_status_callback(self, request: GetMissionStatus.Request,
                              response: GetMissionStatus.Response):
        response.success        = True
        response.status         = self._build_status_msg()
        return response

    # ── A* 경로 계획 ──────────────────────────────────────────────

    def _plan_path(self) -> list:
        """
        모든 목적지를 순서대로 방문하는 웨이포인트 리스트 생성
        각 목적지 쌍 사이를 A*로 연결
        """
        if not self._goals:
            return []

        all_waypoints = []
        # 현재 위치 → 첫 목적지
        start_node = self._graph.nearest_node(
            self._current_x_m * 1000,
            self._current_y_m * 1000
        )
        prev_node = start_node

        for gx, gy, _ in self._goals:
            goal_node = self._graph.nearest_node(gx * 1000, gy * 1000)
            path_segment = self._graph.astar(prev_node, goal_node)
            if not path_segment:
                self.get_logger().error(
                    f'A* 경로 없음: {prev_node} → {goal_node}')
                return []
            # 첫 세그먼트가 아니면 시작 노드 중복 제거
            if all_waypoints:
                path_segment = path_segment[1:]
            all_waypoints.extend(path_segment)
            prev_node = goal_node
            self.get_logger().info(
                f'  경로 세그먼트: {prev_node} → {goal_node} '
                f'({len(path_segment)}개 노드)'
            )

        self.get_logger().info(
            f'전체 경로: {len(all_waypoints)}개 웨이포인트')
        return all_waypoints

    # ── Nav2 이동 제어 ────────────────────────────────────────────

    def _navigate_next(self):
        if self._current_wp_idx >= len(self._waypoints):
            self._on_mission_complete()
            return

        node_id = self._waypoints[self._current_wp_idx]
        x_mm, y_mm = self._graph.node_pose(node_id)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id  = 'map'
        goal_pose.header.stamp     = self.get_clock().now().to_msg()
        goal_pose.pose.position.x  = x_mm * 0.001
        goal_pose.pose.position.y  = y_mm * 0.001
        goal_pose.pose.orientation.w = 1.0

        if self.use_nav2:
            self._send_nav2_goal(goal_pose, node_id)
        else:
            self.get_logger().info(
                f'[robot_{self.robot_id}] 차선 추종 모드: '
                f'→ {node_id} ({x_mm}mm, {y_mm}mm)'
            )
            self._current_wp_idx += 1
            # 재귀 대신 oneshot 타이머로 다음 호출 — 스택오버플로 방지
            self._nav_once_timer = self.create_timer(
                0.05, self._navigate_next_once, callback_group=self._cb_group)

    def _navigate_next_once(self):
        """oneshot 타이머 콜백 — 타이머를 취소·파괴 후 _navigate_next 1회 실행"""
        if self._nav_once_timer is not None:
            self._nav_once_timer.cancel()
            self.destroy_timer(self._nav_once_timer)
            self._nav_once_timer = None
        self._navigate_next()

    def _send_nav2_goal(self, goal_pose: PoseStamped, node_id: str):
        if not self._nav2_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 서버 연결 실패')
            self._mission_state = MissionStatus.FAILED
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self._is_navigating = True
        send_future = self._nav2_client.send_goal_async(
            nav_goal,
            feedback_callback=self._nav2_feedback_callback
        )
        send_future.add_done_callback(
            lambda f: self._nav2_goal_response_callback(f, node_id)
        )
        self.get_logger().info(
            f'[robot_{self.robot_id}] Nav2 목표 전송: '
            f'{node_id} ({goal_pose.pose.position.x:.3f}m, '
            f'{goal_pose.pose.position.y:.3f}m)'
        )

    def _nav2_goal_response_callback(self, future, node_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Nav2 목표 거절: {node_id}')
            self._mission_state = MissionStatus.FAILED
            self._is_navigating = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav2_result_callback(f, node_id)
        )

    def _nav2_result_callback(self, future, node_id):
        self._is_navigating = False
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'[robot_{self.robot_id}] 웨이포인트 도달: {node_id} '
                f'({self._current_wp_idx+1}/{len(self._waypoints)})'
            )
            self._current_wp_idx += 1
            self._navigate_next()
        else:
            self.get_logger().warn(
                f'[robot_{self.robot_id}] Nav2 실패 (status={result.status}): '
                f'{node_id}, 재시도...'
            )
            # 재시도 1회
            self._navigate_next()

    def _nav2_feedback_callback(self, feedback_msg):
        # 거리 정보 디버그 출력 (필요 시)
        pass

    # ── 미션 완료 처리 ────────────────────────────────────────────

    def _on_mission_complete(self):
        if self.loop_mission and self._goals:
            self.get_logger().info(
                f'[robot_{self.robot_id}] 미션 순환 재시작')
            self._current_wp_idx = 0
            self._waypoints = self._plan_path()
            self._navigate_next()
        else:
            self._mission_state = MissionStatus.COMPLETED
            self.get_logger().info(
                f'[robot_{self.robot_id}] 미션 완료: {self._mission_id}')

    # ── 상태 발행 (1Hz) ───────────────────────────────────────────

    def _publish_status(self):
        self._status_pub.publish(self._build_status_msg())

    def _build_status_msg(self) -> MissionStatus:
        msg = MissionStatus()
        msg.stamp_ms         = self._stamp_ms()
        msg.robot_id         = self.robot_id
        msg.mission_state    = self._mission_state
        msg.current_goal_idx = min(
            self._current_wp_idx, 255)
        msg.total_goals      = min(len(self._waypoints), 255)
        if self._waypoints:
            msg.progress_percent = float(
                self._current_wp_idx / len(self._waypoints) * 100.0)
        return msg

    def _stamp_ms(self) -> int:
        ns = self.get_clock().now().nanoseconds
        return int((ns // 1_000_000) & 0xFFFFFFFF)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
