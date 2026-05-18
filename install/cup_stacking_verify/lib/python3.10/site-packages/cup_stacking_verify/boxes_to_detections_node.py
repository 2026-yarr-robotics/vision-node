"""boxes_to_detections_node — bridge depth_digital_twin → cup_stacking_verify.

depth_digital_twin's point_cloud_node publishes per-cup 3D poses as a
``visualization_msgs/MarkerArray`` on ``/digital_twin/boxes`` (frame ``world``,
latched / transient-local QoS).  Per track id it emits, among others:

* ns ``boxes``    — CUBE: ``pose.position`` = box centre, ``scale`` = cup size
* ns ``box_top``  — SPHERE: ``pose.position`` = top-centre world point
                    (the pick target = ``z_base + cup_height``)

cup_stacking_verify's verifier consumes ``vision_msgs/Detection3DArray`` on
``/detected_cups`` with the convention that ``bbox.center.position.z`` is the
TOP of the cup and the cup spans ``[z - size.z, z]`` (see verifier_node and
test_pub).

This node reconstructs per-id boxes from the MarkerArray snapshot
(point_cloud_node republishes the full set every window, plus DELETE /
DELETEALL for evicted tracks) and republishes them as a Detection3DArray.
"""
from __future__ import annotations

from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class BoxesToDetectionsNode(Node):
    def __init__(self) -> None:
        super().__init__('boxes_to_detections_node')

        self.declare_parameter('boxes_topic', '/digital_twin/boxes')
        self.declare_parameter('detections_topic', '/detected_cups')
        # Frame stamped on the outgoing Detection3DArray. depth_digital_twin
        # publishes boxes in `world` (= robot base). Keep the verifier in the
        # same frame so its RViz markers line up with the real detections.
        self.declare_parameter('target_frame', 'world')

        boxes_topic = str(self.get_parameter('boxes_topic').value)
        det_topic = str(self.get_parameter('detections_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)

        # Per track id: {'center': (x,y,z), 'size': (sx,sy,sz),
        #                'quat': (x,y,z,w), 'top': (x,y,z) | None}
        self._boxes: dict[int, dict] = {}
        self._last_stamp = None

        # Match point_cloud_node's latched publisher so we also receive the
        # last snapshot if we start after it.
        boxes_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            MarkerArray, boxes_topic, self._on_boxes, boxes_qos)
        self.pub = self.create_publisher(Detection3DArray, det_topic, 10)

        self.get_logger().info(
            f'boxes_to_detections ready  in={boxes_topic}  '
            f'out={det_topic}  frame={self.target_frame}')

    # ------------------------------------------------------------------
    def _on_boxes(self, msg: MarkerArray) -> None:
        changed = False
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                self._boxes.clear()
                changed = True
                continue
            if m.action == Marker.DELETE:
                if m.ns in ('boxes', 'box_top'):
                    self._boxes.pop(m.id, None)
                    changed = True
                continue
            if m.action != Marker.ADD:
                continue

            if m.ns == 'boxes':
                entry = self._boxes.setdefault(m.id, {})
                entry['center'] = (
                    float(m.pose.position.x),
                    float(m.pose.position.y),
                    float(m.pose.position.z))
                entry['size'] = (
                    float(m.scale.x),
                    float(m.scale.y),
                    float(m.scale.z))
                entry['quat'] = (
                    float(m.pose.orientation.x),
                    float(m.pose.orientation.y),
                    float(m.pose.orientation.z),
                    float(m.pose.orientation.w))
                self._last_stamp = m.header.stamp
                changed = True
            elif m.ns == 'box_top':
                entry = self._boxes.setdefault(m.id, {})
                entry['top'] = (
                    float(m.pose.position.x),
                    float(m.pose.position.y),
                    float(m.pose.position.z))
                changed = True

        if changed:
            self._publish()

    # ------------------------------------------------------------------
    def _publish(self) -> None:
        out = Detection3DArray()
        out.header.frame_id = self.target_frame
        out.header.stamp = (
            self._last_stamp if self._last_stamp is not None
            else self.get_clock().now().to_msg())

        n = 0
        for tid, b in sorted(self._boxes.items()):
            if 'center' not in b or 'size' not in b:
                continue  # box_top arrived before its CUBE; wait for next msg
            cx, cy, cz = b['center']
            sx, sy, sz = b['size']
            # Verifier convention: bbox.center.position.z == TOP of the cup.
            # Prefer the explicit box_top point; else derive from the CUBE
            # (axis-aligned standing cup ⇒ top = center.z + size.z/2).
            if b.get('top') is not None:
                tx, ty, tz = b['top']
            else:
                tx, ty, tz = cx, cy, cz + sz / 2.0

            det = Detection3D()
            det.header = out.header
            det.id = str(tid)
            det.bbox.center.position.x = tx
            det.bbox.center.position.y = ty
            det.bbox.center.position.z = tz
            qx, qy, qz, qw = b.get('quat', (0.0, 0.0, 0.0, 1.0))
            det.bbox.center.orientation.x = qx
            det.bbox.center.orientation.y = qy
            det.bbox.center.orientation.z = qz
            det.bbox.center.orientation.w = qw or 1.0
            det.bbox.size.x = sx
            det.bbox.size.y = sy
            det.bbox.size.z = sz
            out.detections.append(det)
            n += 1

        self.pub.publish(out)
        self.get_logger().info(
            f'→ /detected_cups : {n} cup(s)  ids={sorted(self._boxes)}',
            throttle_duration_sec=1.0)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BoxesToDetectionsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
