"""topic_logger_node — print the vision-node topics as plain text.

Subscribes to everything cup_stacking_verify produces and the bridged input,
then prints a consolidated text snapshot on a fixed cadence so the pipeline
can be followed in a terminal without RViz:

* ``/detected_cups``         (vision_msgs/Detection3DArray) — bridge output
* ``/cup_overlap_ratio``     (std_msgs/Float32)             — verifier output
* ``/cup_occupancy_status``  (std_msgs/Int8)                — verifier output
* ``/virtual_cup_markers``   (visualization_msgs/MarkerArray) — verifier viz

The verifier already logs per-cup detail at callback time; this node gives a
single periodic summary view of the published topics (state, not events).
"""
from __future__ import annotations

from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class TopicLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('topic_logger_node')

        self.declare_parameter('detections_topic', '/detected_cups')
        self.declare_parameter('overlap_topic', '/cup_overlap_ratio')
        self.declare_parameter('status_topic', '/cup_occupancy_status')
        self.declare_parameter('markers_topic', '/virtual_cup_markers')
        self.declare_parameter('period_s', 1.0)

        det_t = str(self.get_parameter('detections_topic').value)
        ovl_t = str(self.get_parameter('overlap_topic').value)
        sta_t = str(self.get_parameter('status_topic').value)
        mrk_t = str(self.get_parameter('markers_topic').value)
        period = float(self.get_parameter('period_s').value)

        # Latest-value cache; the timer renders one text block per period.
        self._detections: list[tuple[str, float, float, float]] = []
        self._overlap: float | None = None
        self._status_last: int | None = None
        self._status_count = 0
        self._virtual_total = 0
        self._virtual_occupied = 0

        self.create_subscription(
            Detection3DArray, det_t, self._on_detections, 10)
        self.create_subscription(Float32, ovl_t, self._on_overlap, 10)
        self.create_subscription(Int8, sta_t, self._on_status, 10)
        self.create_subscription(MarkerArray, mrk_t, self._on_markers, 10)

        self.create_timer(period, self._report)
        self.get_logger().info(
            f'topic_logger ready  det={det_t}  overlap={ovl_t}  '
            f'status={sta_t}  markers={mrk_t}  every {period:.1f}s')

    # ── subscriptions (cache only) ────────────────────────────────────────
    def _on_detections(self, msg: Detection3DArray) -> None:
        self._detections = [
            (d.id or '?',
             d.bbox.center.position.x,
             d.bbox.center.position.y,
             d.bbox.center.position.z)
            for d in msg.detections]

    def _on_overlap(self, msg: Float32) -> None:
        self._overlap = float(msg.data)

    def _on_status(self, msg: Int8) -> None:
        self._status_last = int(msg.data)
        self._status_count += 1

    def _on_markers(self, msg: MarkerArray) -> None:
        total = occ = 0
        for m in msg.markers:
            if m.ns == 'virtual_cups' and m.action == Marker.ADD:
                total += 1
                # verifier colours occupied virtual cups green (g≈1, r≈0)
                if m.color.g > 0.5 and m.color.r < 0.5:
                    occ += 1
        self._virtual_total = total
        self._virtual_occupied = occ

    # ── periodic text report ──────────────────────────────────────────────
    def _report(self) -> None:
        lines = ['──── vision-node topics ────']

        if self._detections:
            lines.append(f'/detected_cups: {len(self._detections)} cup(s)')
            for cid, x, y, z in self._detections:
                lines.append(
                    f'    #{cid:>3}  top=({x:+.3f}, {y:+.3f}, {z:+.3f})')
        else:
            lines.append('/detected_cups: (none)')

        ovl = ('n/a' if self._overlap is None
               else f'{self._overlap:.3f}')
        lines.append(f'/cup_overlap_ratio: max={ovl}')

        if self._status_last is None:
            lines.append('/cup_occupancy_status: (none)')
        else:
            state = 'OCCUPIED' if self._status_last else 'EMPTY'
            lines.append(
                f'/cup_occupancy_status: last={self._status_last} ({state})'
                f'  msgs={self._status_count}')

        lines.append(
            f'/virtual_cup_markers: {self._virtual_occupied}/'
            f'{self._virtual_total} virtual cup(s) occupied')

        self.get_logger().info('\n'.join(lines))


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TopicLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
