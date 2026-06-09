import json
import math
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, Int8, Int32MultiArray, String
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


# Fixed pyramid layout — bottom→top.  The verifier deliberately caps at this
# layout: 3 cups on L1, 2 on L2, 1 on L3 (total 6).  Any observed detections
# beyond layer 2 or beyond a layer's count are dropped from both the slot map
# and the RViz overlay.
LAYER_COUNTS: tuple[int, ...] = (3, 2, 1)

# Slot naming per layer cup count. Bottom layer = level 1.
# Indices are left→right within the layer (verifier sorts by x).
_SLOT_NAMES_BY_COUNT: dict[int, list[str]] = {
    1: ['T'],
    2: ['L', 'R'],
    3: ['L', 'M', 'R'],
}


def _slot_name(level: int, pos_idx: int, layer_count: int) -> str:
    """e.g. (1, 0, 3) → 'L1_L'; (3, 0, 1) → 'L3_T'."""
    suffixes = _SLOT_NAMES_BY_COUNT.get(
        layer_count, [f'p{i}' for i in range(layer_count)])
    suffix = suffixes[pos_idx] if pos_idx < len(suffixes) else f'p{pos_idx}'
    return f'L{level}_{suffix}'


def _build_slot_keys(layer_counts=LAYER_COUNTS) -> list[str]:
    """All slot keys for the layout, layer 1 → top."""
    keys: list[str] = []
    for layer_idx, count in enumerate(layer_counts):
        for pos_i in range(count):
            keys.append(_slot_name(layer_idx + 1, pos_i, count))
    return keys


class CupOccupancyNode(Node):
    def __init__(self):
        super().__init__('cup_occupancy_verifier')

        # 1. 스펙 및 파라미터
        # 가로 간격/층 높이를 FastAPI RobotDomain pyramid 배치 기하에 맞춤
        # (server/server/domains/robot.py: PYRAMID_CUP_SPACING=0.078,
        #  PYRAMID_LAYER_HEIGHT=0.093). cup_ref_w 가 슬롯 가로 간격이자 박스 폭,
        #  층 수직 피치 = cup_ref_h + layer_gap = 0.086 + 0.007 = 0.093.
        self.cup_ref_w = 0.078    # = PYRAMID_CUP_SPACING (슬롯 가로 간격, base+X)
        self.cup_ref_d = 0.070
        self.cup_ref_h = 0.086
        self.cup_ref_vol = self.cup_ref_w * self.cup_ref_d * self.cup_ref_h
        self.layer_gap  = 0.007   # 층 피치 0.093 = cup_ref_h(0.086)+layer_gap
        self.box_margin = 0.010   # 박스 시각화 여백 — 인접 컵 사이 간격 표현 (m)

        # New geometry: cp = L1_M position (centre of the bottom layer),
        # degree = row orientation in degrees (base +X = 0°, CCW positive
        # around base +Z).  Replaces the old p_start (L1_L) + v_dir (3D unit)
        # pair so the pose can be set with the same convention the API uses.
        self.declare_parameter('cp', [0.5, 0.0, 0.1])
        # 기본 90° = FastAPI RobotDomain DEFAULT_PYRAMID_DEGREE 와 일치
        # (행을 base +Y 로 펼침). 0° 이면 +X 라 Pyramid API 와 90° 어긋남.
        self.declare_parameter('degree', 90.0)
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('target_index', 0)
        # Frame for all RViz markers. depth_digital_twin bridges detections in
        # `world` (= robot base), so default to `world` to keep this node's
        # markers aligned with the real cup detections without a world↔base_link
        # TF. Override to `base_link` if such a TF exists.
        self.declare_parameter('target_frame', 'world')
        # Render/boundary publish rate. cp/degree are re-read every tick so
        # the pose_tuner UI applies in real time.  The pyramid layout itself
        # is fixed by LAYER_COUNTS at module scope and NOT parameterisable.
        self.declare_parameter('publish_rate_hz', 10.0)
        # Detected-cup overlay is shown only while detections are this fresh.
        self.declare_parameter('detection_timeout_s', 1.5)
        # Length of the degree-direction arrow marker (m, in base XY plane).
        self.declare_parameter('arrow_length', 0.25)

        self.frame_id = str(self.get_parameter('target_frame').value)

        # 2. Pub/Sub
        self.sub_detection = self.create_subscription(
            Detection3DArray, '/detected_cups', self.detection_callback, 10)

        self.pub_status = self.create_publisher(Int8, '/cup_occupancy_status', 10)
        self.pub_ratio = self.create_publisher(Float32, '/cup_overlap_ratio', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/virtual_cup_markers', 10)
        # Slot-level publications (system_state_aggregator + depth exclusion).
        # /stack         — JSON {slot_name: <color>|null} for every slot in the
        #                  configured layout. Sticky: emitted every render tick
        #                  even with no detections (slots all null).
        # /stack_track_ids — depth track ids currently occupying ANY slot, so
        #                  depth's /cups_on_table can subtract them.
        self.pub_stack = self.create_publisher(String, '/stack', 10)
        self.pub_stack_ids = self.create_publisher(
            Int32MultiArray, '/stack_track_ids', 10)
        # Slot occupancy threshold (separate from `threshold` which gates the
        # overall Int8 status). Default lower so partial fits still register.
        self.declare_parameter('slot_occupancy_overlap_min', 0.4)
        # In-flight transient 억제: 슬롯이 같은 track id 로 ratio>=overlap_min 을
        # slot_confirm_frames 연속(관측 간격 <= slot_confirm_timeout_s) 유지할 때만
        # /stack 에 occupied 로 반영(공중 컵의 일시 L2/L3 오검출 방지). 사라짐은 즉시
        # 반영(외란/유실 빠르게). frames=1 이면 즉시(기존 동작, replay 테스트용).
        self.declare_parameter('slot_confirm_frames', 3)
        self.declare_parameter('slot_confirm_timeout_s', 0.5)
        self._stack_slot_keys = _build_slot_keys(LAYER_COUNTS)
        self._slot_streak = {}  # slot_key -> {tid, count, last_t}

        # Latest detections, rendered by the timer (decoupled from arrival rate
        # so the boundary/pose markers are published even with no detections).
        self._last_msg = None
        self._last_stamp_s = 0.0

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._render)

        # ── Runtime geometry sync: keep cp/degree == FastAPI pyramid config ──
        # The slot boxes are anchored at cp (= L1_M) with rotation degree; the
        # robot's actual placement geometry is owned by the FastAPI server
        # (GET /api/robot/config/pyramid → center{x,y} + degree). If they drift
        # the placed cup falls outside the slot box and /stack never flips to
        # occupied, stalling the LLM loop. A background poller (order- and
        # boot-timing-independent, unlike a launch-time fetch) tracks the API
        # value and self-heals once the server is up.
        self.declare_parameter('sync_pyramid_geometry', False)
        self.declare_parameter(
            'pyramid_config_url',
            'https://yarr-api-31.simplyimg.com/api/robot/config/pyramid')
        self.declare_parameter('sync_poll_period_s', 5.0)
        # cp_z = perceived L1 cup-top height in world frame (NOT the API gripper
        # place_z). The poll only overwrites cp.x/cp.y + degree, keeping this z.
        self.declare_parameter('cp_z', float(self.get_parameter('cp').value[2]))
        if bool(self.get_parameter('sync_pyramid_geometry').value):
            url = str(self.get_parameter('pyramid_config_url').value)
            period = max(1.0, float(self.get_parameter('sync_poll_period_s').value))
            cp_z = float(self.get_parameter('cp_z').value)
            if url:
                threading.Thread(
                    target=self._geometry_sync_loop,
                    args=(url, period, cp_z), daemon=True).start()
                self.get_logger().info(
                    f'[geometry-sync] polling {url} every {period:.0f}s')

        self.get_logger().info(
            "Verifier started — boundary/pose markers always published "
            f"@ {rate:.0f} Hz (frame={self.frame_id})")

    def _geometry_sync_loop(self, url: str, period: float, cp_z: float) -> None:
        """Background: poll the FastAPI pyramid config and mirror center/degree
        into this node's cp/degree params so slots track where cups are placed.

        Runs off the executor thread; HTTP never blocks rendering. Only re-sets
        params when the value changes (so a manual pose_tuner tweak isn't fought
        every tick unless the API differs). Cloudflare 403s the default urllib
        User-Agent, so present a curl-like one."""
        req = urllib.request.Request(url, headers={'User-Agent': 'curl/7.81.0'})
        last = None
        while rclpy.ok():
            try:
                with urllib.request.urlopen(req, timeout=3) as resp:
                    data = json.loads(resp.read())
                cx = float(data['center']['x'])
                cy = float(data['center']['y'])
                deg = float(data['degree'])
                key = (round(cx, 4), round(cy, 4), round(deg, 2))
                if key != last:
                    self.set_parameters([
                        Parameter('cp', Parameter.Type.DOUBLE_ARRAY,
                                  [cx, cy, cp_z]),
                        Parameter('degree', Parameter.Type.DOUBLE, deg),
                    ])
                    self.get_logger().info(
                        f'[geometry-sync] cp=[{cx:.3f},{cy:.3f},{cp_z:.3f}] '
                        f'degree={deg:.1f}')
                    last = key
            except Exception as exc:  # noqa: BLE001 — unreachable/parse → retry
                self.get_logger().warn(
                    f'[geometry-sync] fetch failed: {exc}',
                    throttle_duration_sec=15.0)
            time.sleep(period)

    # ── 가상 박스 기하 ─────────────────────────────────────────────────────
    def get_virtual_box(self, index: int, layer: int = 0):
        """AABB for slot (layer, index) in the fixed [3,2,1] layout.

        Geometry (cp = L1_M, degree = base +X CCW around base +Z):
          • Row direction  d = (cos θ, sin θ, 0),  θ = radians(degree)
          • Within a layer of N cups, slot i is placed at offset
                (i − (N−1)/2) · cup_w  along d
            so layer L1 (N=3) → −w, 0, +w  (L, M, R)
               layer L2 (N=2) → −0.5w, +0.5w  (L, R nested between L1)
               layer L3 (N=1) → 0          (T directly above L1_M)
          • Vertical: cp.z + layer · (cup_h + layer_gap)
        Box itself stays AABB (matching depth_digital_twin's box convention).
        """
        cp = self.get_parameter('cp').value
        deg = float(self.get_parameter('degree').value)
        theta = math.radians(deg)
        ux, uy = math.cos(theta), math.sin(theta)

        if 0 <= layer < len(LAYER_COUNTS):
            n = LAYER_COUNTS[layer]
        else:
            n = 1                               # graceful for callers that
                                                # somehow probe layer ≥ 3
        offset = (index - (n - 1) / 2.0) * self.cup_ref_w

        c_x = float(cp[0]) + offset * ux
        c_y = float(cp[1]) + offset * uy
        c_z = float(cp[2]) + layer * (self.cup_ref_h + self.layer_gap)

        return ([c_x - self.cup_ref_w/2, c_y - self.cup_ref_d/2,
                 c_z - self.cup_ref_h],
                [c_x + self.cup_ref_w/2, c_y + self.cup_ref_d/2, c_z])

    def calculate_overlap_ratio(self, v_min, v_max, d_min, d_max):
        dx = max(0, min(v_max[0], d_max[0]) - max(v_min[0], d_min[0]))
        dy = max(0, min(v_max[1], d_max[1]) - max(v_min[1], d_min[1]))
        dz = max(0, min(v_max[2], d_max[2]) - max(v_min[2], d_min[2]))
        return (dx * dy * dz) / self.cup_ref_vol

    def _compute_slots(self, msg, threshold):
        """고정 6슬롯 각각에 가장 많이 겹치는 detection을 greedy(ratio 내림차순)로
        배정한다.

        기존 z-layer 그룹핑 → x정렬 → 레이어별 cap 방식은 주변 테이블 컵이 x순위로
        앞서면 실제로 슬롯에 놓인 컵을 엉뚱한 박스에 대입하거나 cap에 걸려 drop시켜,
        놓인 컵이 있어도 /stack이 occupied로 바뀌지 않는 구조적 실패가 있었다.
        여기서는 LAYER_COUNTS=(3,2,1)의 6개 고정 슬롯마다 모든 detection과 3D
        overlap을 계산하고, ratio가 큰 순서로 슬롯·detection을 각각 한 번씩만
        배정한다(상호배제). 화면에 테이블 컵이 몇 개 있든 각 슬롯은 자신과 실제로
        가장 많이 겹치는 컵만 가져간다.

        Returns (records, max_ratio, slot_debug):
          records   — 배정된 슬롯별 dict(스키마: layer/pos/detection/v_min/v_max/
                      ratio/occupied). occupied 는 marker용 threshold(0.6).
          max_ratio — 배정된 슬롯 중 최대 overlap.
          slot_debug — slot_key -> (detection|None, ratio). 미점유 포함 로그용.
        """
        dets = []
        for det in msg.detections:
            pos = det.bbox.center.position
            size = det.bbox.size
            d_min = [pos.x - size.x/2, pos.y - size.y/2, pos.z - size.z]
            d_max = [pos.x + size.x/2, pos.y + size.y/2, pos.z]
            dets.append((det, d_min, d_max))

        slots = []  # (layer_idx, pos_i, slot_key, v_min, v_max)
        for layer_idx, count in enumerate(LAYER_COUNTS):
            for pos_i in range(count):
                v_min, v_max = self.get_virtual_box(pos_i, layer_idx)
                slots.append((layer_idx, pos_i,
                              _slot_name(layer_idx + 1, pos_i, count),
                              v_min, v_max))

        candidates = []  # (ratio, slot_index, det_index), overlap>0 only
        raw_best = {}    # slot_index -> (ratio, det_index): per-slot max, pre-greedy
        for si, (_li, _pi, _sk, v_min, v_max) in enumerate(slots):
            for di, (_det, d_min, d_max) in enumerate(dets):
                ratio = self.calculate_overlap_ratio(v_min, v_max, d_min, d_max)
                if ratio > 0.0:
                    candidates.append((ratio, si, di))
                    if si not in raw_best or ratio > raw_best[si][0]:
                        raw_best[si] = (ratio, di)
        candidates.sort(key=lambda c: c[0], reverse=True)

        used_slots = set()
        used_dets = set()
        best_by_slot = {}  # slot_index -> (ratio, det_index)
        for ratio, si, di in candidates:
            if si in used_slots or di in used_dets:
                continue
            used_slots.add(si)
            used_dets.add(di)
            best_by_slot[si] = (ratio, di)

        records = []
        slot_debug = {}
        max_ratio = 0.0
        for si, (layer_idx, pos_i, slot_key, v_min, v_max) in enumerate(slots):
            rb = raw_best.get(si)
            raw_det = dets[rb[1]][0] if rb else None
            raw_ratio = rb[0] if rb else 0.0
            assigned = best_by_slot.get(si)
            assigned_ratio = assigned[0] if assigned else 0.0
            slot_debug[slot_key] = (raw_det, raw_ratio, assigned_ratio)
            if assigned is None:
                continue
            ratio, di = assigned
            det = dets[di][0]
            max_ratio = max(max_ratio, ratio)
            records.append({
                'layer': layer_idx, 'pos': pos_i, 'detection': det,
                'v_min': v_min, 'v_max': v_max, 'ratio': ratio,
                'occupied': ratio > threshold,
            })
        return records, max_ratio, slot_debug

    def _log_slot_debug(self, slot_debug: dict) -> None:
        """슬롯별 best detection/ratio를 throttle 걸어 한 줄로 남긴다.
        assigned 기준은 /stack 임계(slot_occupancy_overlap_min, 기본 0.4)."""
        min_overlap = float(
            self.get_parameter('slot_occupancy_overlap_min').value)
        parts = []
        for slot_key in self._stack_slot_keys:
            raw_det, raw_ratio, assigned_ratio = slot_debug.get(
                slot_key, (None, 0.0, 0.0))
            if raw_det is None:
                parts.append(f'{slot_key}=None(0.00,no)')
                continue
            try:
                did = int(raw_det.id)
            except (ValueError, TypeError):
                did = raw_det.id
            ok = 'yes' if assigned_ratio >= min_overlap else 'no'
            parts.append(f'{slot_key}=#{did}({raw_ratio:.2f},{ok})')
        self.get_logger().info(
            'slot match | ' + '  '.join(parts), throttle_duration_sec=2.0)

    # ── 콜백: status/ratio 즉시 발행, 마커는 타이머가 렌더 ──────────────────
    def detection_callback(self, msg):
        self._last_msg = msg
        self._last_stamp_s = self.get_clock().now().nanoseconds * 1e-9

        threshold = self.get_parameter('threshold').value
        records, max_ratio, slot_debug = self._compute_slots(msg, threshold)
        self.pub_status.publish(Int8(data=1 if max_ratio > threshold else 0))
        self.pub_ratio.publish(Float32(data=max_ratio))
        self._publish_stack(records)
        self._log_slot_debug(slot_debug)

    # ── /stack + /stack_track_ids 발행 ────────────────────────────────────
    def _publish_stack(self, records: list[dict]) -> None:
        """Publish /stack (JSON {slot: color|null}) and /stack_track_ids.

        Slot keys are the fixed LAYER_COUNTS layout, so the emitted schema is
        constant — empty slots stay `null`.  A record contributes only when
        its overlap clears `slot_occupancy_overlap_min` (default 0.4).
        `_compute_slots` returns at most one record per slot, but we
        re-check the bounds defensively here.
        """
        min_overlap = float(
            self.get_parameter('slot_occupancy_overlap_min').value)
        confirm_frames = int(
            self.get_parameter('slot_confirm_frames').value)
        confirm_timeout = float(
            self.get_parameter('slot_confirm_timeout_s').value)
        now = self.get_clock().now().nanoseconds * 1e-9

        # 이번 프레임에 overlap_min 이상으로 채워진 슬롯 후보(confirm 전 raw).
        raw_occupied: dict[str, tuple] = {}
        for r in records:
            if r['ratio'] < min_overlap:
                continue
            layer_idx, pos_i = r['layer'], r['pos']
            if layer_idx >= len(LAYER_COUNTS):
                continue
            vcount = LAYER_COUNTS[layer_idx]
            if pos_i >= vcount:
                continue
            slot = _slot_name(layer_idx + 1, pos_i, vcount)
            try:
                tid = int(r['detection'].id)
            except (ValueError, TypeError):
                tid = -1
            raw_occupied[slot] = (self._color_of(r['detection']), tid)

        # N-frame confirm: 등장은 같은 track id 가 confirm_frames 연속 유지될 때만,
        # 사라짐은 즉시(streak drop).
        slot_map: dict[str, str | None] = {
            k: None for k in self._stack_slot_keys}
        track_ids: list[int] = []
        for slot in self._stack_slot_keys:
            cand = raw_occupied.get(slot)
            if cand is None:
                self._slot_streak.pop(slot, None)
                continue
            color, tid = cand
            st = self._slot_streak.get(slot)
            if (st is not None and st['tid'] == tid
                    and now - st['last_t'] <= confirm_timeout):
                st['count'] += 1
                st['last_t'] = now
            else:
                st = {'tid': tid, 'count': 1, 'last_t': now}
                self._slot_streak[slot] = st
            if st['count'] >= confirm_frames:
                slot_map[slot] = color
                if tid >= 0:
                    track_ids.append(tid)

        self.pub_stack.publish(
            String(data=json.dumps(slot_map, ensure_ascii=False)))
        ids_msg = Int32MultiArray()
        ids_msg.data = sorted(set(track_ids))
        self.pub_stack_ids.publish(ids_msg)

    @staticmethod
    def _color_of(det) -> str:
        """Color label forwarded by boxes_to_detections_node via
        results[0].hypothesis.class_id. 'unknown' if not propagated."""
        try:
            label = det.results[0].hypothesis.class_id
        except (AttributeError, IndexError):
            return 'unknown'
        return label or 'unknown'

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
        """cp 구 + degree 화살표 + 라벨 — 항상 표시.

        Sphere sits at cp (= L1_M).  Arrow shows the +X row direction in the
        base XY plane (length = arrow_length parameter)."""
        cp = self.get_parameter('cp').value
        deg = float(self.get_parameter('degree').value)
        theta = math.radians(deg)
        ux, uy = math.cos(theta), math.sin(theta)
        out = []

        sphere = self._hdr(Marker())
        sphere.ns = "pose_origin"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(cp[0])
        sphere.pose.position.y = float(cp[1])
        sphere.pose.position.z = float(cp[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.035
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.4, 0.0, 1.0
        out.append(sphere)

        L = float(self.get_parameter('arrow_length').value)
        arrow = self._hdr(Marker())
        arrow.ns = "pose_dir"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.points.append(
            Point(x=float(cp[0]), y=float(cp[1]), z=float(cp[2])))
        arrow.points.append(Point(
            x=float(cp[0] + ux * L), y=float(cp[1] + uy * L),
            z=float(cp[2])))
        arrow.scale.x = 0.012   # shaft dia
        arrow.scale.y = 0.025   # head dia
        arrow.scale.z = 0.04    # head len
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.4, 0.0, 1.0
        out.append(arrow)

        label = self.create_text_marker(
            [float(cp[0]), float(cp[1]), float(cp[2]) + 0.02],
            f'cp = L1_M ({cp[0]:.3f}, {cp[1]:.3f}, {cp[2]:.3f})\n'
            f'degree {deg:.1f}°',
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

        # 2) 타겟 경계 — 고정 LAYER_COUNTS 피라미드 (상시, 검출 없어도 표시)
        for layer_idx, n in enumerate(LAYER_COUNTS):
            for pos_i in range(n):
                v_min, v_max = self.get_virtual_box(pos_i, layer_idx)
                ma.markers.append(self._boundary_outline_marker(
                    v_min, v_max, layer_idx * 100 + pos_i))

        # 3) 검출 오버레이 (최근 검출이 있을 때만)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        timeout = float(self.get_parameter('detection_timeout_s').value)
        fresh = (self._last_msg is not None
                 and self._last_msg.detections
                 and now_s - self._last_stamp_s <= timeout)
        # Heartbeat: when no fresh detection, still emit an all-null /stack so
        # downstream consumers see a steady schema, not topic silence.
        if not fresh:
            self._publish_stack([])
        if fresh:
            threshold = self.get_parameter('threshold').value
            records, _, _ = self._compute_slots(self._last_msg, threshold)
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
