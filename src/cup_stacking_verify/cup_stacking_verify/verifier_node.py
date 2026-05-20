import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class CupOccupancyNode(Node):
    def __init__(self):
        super().__init__('cup_occupancy_verifier')

        # 1. 스펙 및 파라미터
        self.cup_ref_w = 0.070
        self.cup_ref_d = 0.070
        self.cup_ref_h = 0.086
        self.cup_ref_vol = self.cup_ref_w * self.cup_ref_d * self.cup_ref_h
        self.layer_gap  = 0.002   # 레이어 간 수직 간격 (m)
        self.box_margin = 0.010   # 박스 시각화 여백 — 인접 컵 사이 간격 표현 (m)

        self.declare_parameter('p_start', [0.5, 0.0, 0.1])
        self.declare_parameter('v_dir', [1.0, 0.0, 0.0])
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('target_index', 0)
        # Frame for all RViz markers. depth_digital_twin bridges detections in
        # `world` (= robot base), so default to `world` to keep this node's
        # markers aligned with the real cup detections without a world↔base_link
        # TF. Override to `base_link` if such a TF exists.
        self.declare_parameter('target_frame', 'world')
        # Target stack layout (cups per layer, bottom → top). Drawn as an
        # always-on boundary even when no cups are detected.
        self.declare_parameter('virtual_counts', [3, 2, 1])
        # Render/boundary publish rate. p_start/v_dir are re-read every tick so
        # the pose_tuner UI applies in real time.
        self.declare_parameter('publish_rate_hz', 10.0)
        # Detected-cup overlay is shown only while detections are this fresh.
        self.declare_parameter('detection_timeout_s', 1.5)
        # Length of the v_dir arrow marker (m).
        self.declare_parameter('arrow_length', 0.25)

        self.frame_id = str(self.get_parameter('target_frame').value)

        # 2. Pub/Sub
        self.sub_detection = self.create_subscription(
            Detection3DArray, '/detected_cups', self.detection_callback, 10)

        self.pub_status = self.create_publisher(Int8, '/cup_occupancy_status', 10)
        self.pub_ratio = self.create_publisher(Float32, '/cup_overlap_ratio', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/virtual_cup_markers', 10)

        # Latest detections, rendered by the timer (decoupled from arrival rate
        # so the boundary/pose markers are published even with no detections).
        self._last_msg = None
        self._last_stamp_s = 0.0

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._render)

        self.get_logger().info(
            "Verifier started — boundary/pose markers always published "
            f"@ {rate:.0f} Hz (frame={self.frame_id})")

    # ── 가상 박스 기하 ─────────────────────────────────────────────────────
    def get_virtual_box(self, index, layer=0):
        p_start = self.get_parameter('p_start').value
        v_dir = self.get_parameter('v_dir').value
        mag = (v_dir[0]**2 + v_dir[1]**2 + v_dir[2]**2)**0.5
        if mag < 1e-9:
            unit_dir = [1.0, 0.0, 0.0]
        else:
            unit_dir = [v_dir[0]/mag, v_dir[1]/mag, v_dir[2]/mag]

        layer_height = self.cup_ref_h + self.layer_gap
        # 피라미드 구조: 상위 레이어는 진행 방향으로 cup_ref_w/2씩 오프셋
        offset = (index + layer * 0.5) * self.cup_ref_w
        c_x = p_start[0] + offset * unit_dir[0]
        c_y = p_start[1] + offset * unit_dir[1]
        c_z = p_start[2] + offset * unit_dir[2] + layer * layer_height

        v_min = [c_x - self.cup_ref_w/2, c_y - self.cup_ref_d/2, c_z - self.cup_ref_h]
        v_max = [c_x + self.cup_ref_w/2, c_y + self.cup_ref_d/2, c_z]
        return v_min, v_max

    def calculate_overlap_ratio(self, v_min, v_max, d_min, d_max):
        dx = max(0, min(v_max[0], d_max[0]) - max(v_min[0], d_min[0]))
        dy = max(0, min(v_max[1], d_max[1]) - max(v_min[1], d_min[1]))
        dz = max(0, min(v_max[2], d_max[2]) - max(v_min[2], d_min[2]))
        return (dx * dy * dz) / self.cup_ref_vol

    def _compute_layers(self, msg, threshold):
        """검출을 레이어로 묶고 가상 박스와 overlap 계산.
        Returns (records, max_ratio, layer_sizes)."""
        z_tol = self.cup_ref_h / 2
        layers = []  # 같은 레이어의 detection 목록
        for detection in msg.detections:
            z = detection.bbox.center.position.z
            placed = False
            for group in layers:
                if abs(group[0].bbox.center.position.z - z) < z_tol:
                    group.append(detection)
                    placed = True
                    break
            if not placed:
                layers.append([detection])

        layers.sort(key=lambda g: g[0].bbox.center.position.z)  # bottom→top

        records = []
        max_ratio = 0.0
        for layer_idx, layer_cups in enumerate(layers):
            layer_cups.sort(key=lambda d: d.bbox.center.position.x)
            for pos_i, detection in enumerate(layer_cups):
                pos = detection.bbox.center.position
                size = detection.bbox.size
                d_min = [pos.x - size.x/2, pos.y - size.y/2, pos.z - size.z]
                d_max = [pos.x + size.x/2, pos.y + size.y/2, pos.z]
                v_min, v_max = self.get_virtual_box(pos_i, layer_idx)
                ratio = self.calculate_overlap_ratio(v_min, v_max, d_min, d_max)
                max_ratio = max(max_ratio, ratio)
                records.append({
                    'layer': layer_idx, 'pos': pos_i, 'detection': detection,
                    'v_min': v_min, 'v_max': v_max, 'ratio': ratio,
                    'occupied': ratio > threshold,
                })
        return records, max_ratio, [len(g) for g in layers]

    # ── 콜백: status/ratio 즉시 발행, 마커는 타이머가 렌더 ──────────────────
    def detection_callback(self, msg):
        self._last_msg = msg
        self._last_stamp_s = self.get_clock().now().nanoseconds * 1e-9

        threshold = self.get_parameter('threshold').value
        records, max_ratio, layer_sizes = self._compute_layers(msg, threshold)
        for r in records:
            self.pub_status.publish(Int8(data=1 if r['occupied'] else 0))
        self.pub_ratio.publish(Float32(data=max_ratio))
        self.get_logger().info(
            f'Layers detected: {layer_sizes} | Max overlap: {max_ratio:.2f}')

    # ── 마커 빌더 ──────────────────────────────────────────────────────────
    def _hdr(self, marker):
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        return marker

    def create_marker(self, v_min, v_max, is_occupied, index, ns="virtual_cups"):
        """검출 기반 점유/미점유 가상 컵 박스(채워진 CUBE)."""
        m = self._hdr(Marker())
        m.ns = ns
        m.id = index
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = (v_min[0] + v_max[0]) / 2
        m.pose.position.y = (v_min[1] + v_max[1]) / 2
        m.pose.position.z = (v_min[2] + v_max[2]) / 2
        m.pose.orientation.w = 1.0
        m.scale.x = self.cup_ref_w - 2 * self.box_margin
        m.scale.y = self.cup_ref_d - 2 * self.box_margin
        m.scale.z = self.cup_ref_h
        if is_occupied:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.5
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.3
        return m

    def create_text_marker(self, position, text, index, ns="overlap_text"):
        m = self._hdr(Marker())
        m.ns = ns
        m.id = index
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = position[0]
        m.pose.position.y = position[1]
        m.pose.position.z = position[2] + 0.1
        m.pose.orientation.w = 1.0
        m.scale.z = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0
        m.text = text
        return m

    def create_detected_cup_marker(self, detection, index):
        m = self._hdr(Marker())
        m.ns = "detected_cups"
        m.id = index
        m.type = Marker.CUBE
        m.action = Marker.ADD
        pos = detection.bbox.center.position
        size = detection.bbox.size
        m.pose.position.x = pos.x
        m.pose.position.y = pos.y
        # Detection z는 top-center 기준 → 시각화 시 center z로 변환
        m.pose.position.z = pos.z - size.z / 2
        m.pose.orientation.w = 1.0
        m.scale.x = size.x
        m.scale.y = size.y
        m.scale.z = size.z
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 0.5
        return m

    def _boundary_outline_marker(self, v_min, v_max, index):
        """항상 표시되는 타겟 컵 경계(와이어프레임 LINE_LIST)."""
        m = self._hdr(Marker())
        m.ns = "virtual_boundary"
        m.id = index
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.003
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 0.9
        x0, y0, z0 = v_min[0], v_min[1], v_min[2]
        x1, y1, z1 = v_max[0], v_max[1], v_max[2]
        c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
             (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]
        for a, b in edges:
            m.points.append(Point(x=c[a][0], y=c[a][1], z=c[a][2]))
            m.points.append(Point(x=c[b][0], y=c[b][1], z=c[b][2]))
        return m

    def _pose_markers(self):
        """p_start(구) + v_dir(화살표) + 라벨 — 항상 표시."""
        p = self.get_parameter('p_start').value
        d = self.get_parameter('v_dir').value
        out = []

        sphere = self._hdr(Marker())
        sphere.ns = "pose_origin"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(p[0])
        sphere.pose.position.y = float(p[1])
        sphere.pose.position.z = float(p[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.035
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.4, 0.0, 1.0
        out.append(sphere)

        mag = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        if mag > 1e-9:
            L = float(self.get_parameter('arrow_length').value)
            ux, uy, uz = d[0]/mag, d[1]/mag, d[2]/mag
            arrow = self._hdr(Marker())
            arrow.ns = "pose_dir"
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            arrow.points.append(Point(
                x=float(p[0] + ux*L), y=float(p[1] + uy*L), z=float(p[2] + uz*L)))
            arrow.scale.x = 0.012   # shaft dia
            arrow.scale.y = 0.025   # head dia
            arrow.scale.z = 0.04    # head len
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.4, 0.0, 1.0
            out.append(arrow)

        label = self.create_text_marker(
            [float(p[0]), float(p[1]), float(p[2]) + 0.02],
            f'p_start ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})\n'
            f'v_dir ({d[0]:.2f}, {d[1]:.2f}, {d[2]:.2f})',
            0, ns="pose_text")
        out.append(label)
        return out

    # ── 렌더 타이머 ────────────────────────────────────────────────────────
    def _render(self):
        ma = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # 1) 위치/방향 마커 (상시)
        ma.markers.extend(self._pose_markers())

        # 2) 타겟 경계 — virtual_counts 피라미드 (상시, 검출 없어도 표시)
        counts = list(self.get_parameter('virtual_counts').value)
        for layer_idx, n in enumerate(counts):
            for pos_i in range(int(n)):
                v_min, v_max = self.get_virtual_box(pos_i, layer_idx)
                ma.markers.append(self._boundary_outline_marker(
                    v_min, v_max, layer_idx * 100 + pos_i))

        # 3) 검출 오버레이 (최근 검출이 있을 때만)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        timeout = float(self.get_parameter('detection_timeout_s').value)
        if (self._last_msg is not None
                and self._last_msg.detections
                and now_s - self._last_stamp_s <= timeout):
            threshold = self.get_parameter('threshold').value
            records, _, _ = self._compute_layers(self._last_msg, threshold)
            for r in records:
                idx = r['pos'] + r['layer'] * 100
                ma.markers.append(self.create_marker(
                    r['v_min'], r['v_max'], r['occupied'], 300 + idx))
                pos = r['detection'].bbox.center.position
                size = r['detection'].bbox.size
                ma.markers.append(self.create_text_marker(
                    [pos.x, pos.y, pos.z + size.z / 2],
                    f"{r['ratio']:.2f}", 200 + idx))
                ma.markers.append(self.create_detected_cup_marker(
                    r['detection'], idx))

        self.pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CupOccupancyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
