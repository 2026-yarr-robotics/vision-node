import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

class CupOccupancyNode(Node):
    def __init__(self):
        super().__init__('cup_occupancy_verifier')

        # 1. 스펙 및 파라미터
        self.cup_ref_w = 0.078
        self.cup_ref_d = 0.078
        self.cup_ref_h = 0.095
        self.cup_ref_vol = self.cup_ref_w * self.cup_ref_d * self.cup_ref_h
        
        self.declare_parameter('p_start', [0.5, 0.0, 0.1])
        self.declare_parameter('v_dir', [1.0, 0.0, 0.0])
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('target_index', 0)

        # 2. Pub/Sub
        self.sub_detection = self.create_subscription(
            Detection3DArray, '/detected_cups', self.detection_callback, 10)
        
        self.pub_status = self.create_publisher(Int8, '/cup_occupancy_status', 10)
        self.pub_ratio = self.create_publisher(Float32, '/cup_overlap_ratio', 10)
        
        # RViz 시각화를 위한 퍼블리셔 추가
        self.pub_marker = self.create_publisher(MarkerArray, '/virtual_cup_markers', 10)

        self.get_logger().info("Verifier Node with RViz visualization started.")

    def create_marker(self, v_min, v_max, is_occupied, index, overlap_ratio=None):
        """RViz에 띄울 박스 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "virtual_cups"
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Top Center 기반 박스의 중심점 계산
        marker.pose.position.x = (v_min[0] + v_max[0]) / 2
        marker.pose.position.y = (v_min[1] + v_max[1]) / 2
        marker.pose.position.z = (v_min[2] + v_max[2]) / 2
        
        marker.scale.x = self.cup_ref_w
        marker.scale.y = self.cup_ref_d
        marker.scale.z = self.cup_ref_h
        
        # 색상: 점유 시 초록(Green), 미점유 시 빨강(Red)
        if is_occupied:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.5
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.3
            
        return marker

    def create_text_marker(self, position, text, index):
        """Overlap 비율을 표시하는 텍스트 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "overlap_text"
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.1  # 박스 위에 표시
        marker.scale.z = 0.05  # 텍스트 크기
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        marker.text = text
        return marker

    def create_detected_cup_marker(self, detection, index):
        """검출된 컵을 박스로 시각화하는 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_cups"
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        pos = detection.bbox.center.position
        size = detection.bbox.size
        marker.pose.position.x = pos.x
        marker.pose.position.y = pos.y
        # Detection z는 top-center 기준이므로 시각화할 때는 center z로 변환
        marker.pose.position.z = pos.z - size.z / 2
        marker.scale.x = size.x
        marker.scale.y = size.y
        marker.scale.z = size.z
        
        # 파란색으로 검출된 컵 표시
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 0.5
            
        return marker

    def get_virtual_box(self, index, layer=0):
        p_start = self.get_parameter('p_start').value
        v_dir = self.get_parameter('v_dir').value
        mag = (v_dir[0]**2 + v_dir[1]**2 + v_dir[2]**2)**0.5
        unit_dir = [v_dir[0]/mag, v_dir[1]/mag, v_dir[2]/mag]

        # 레이어 높이 추가 계산
        layer_height = self.cup_ref_h + 0.02  # 각 레이어 간 간격 추가
        # 피라미드 구조: 상위 레이어는 x방향으로 cup_ref_w/2씩 오프셋
        offset = (index + layer * 0.5) * self.cup_ref_w
        c_x = p_start[0] + offset * unit_dir[0]
        c_y = p_start[1] + offset * unit_dir[1]
        c_z = p_start[2] + offset * unit_dir[2] + layer * layer_height

        v_min = [c_x - self.cup_ref_w/2, c_y - self.cup_ref_d/2, c_z - self.cup_ref_h]
        v_max = [c_x + self.cup_ref_w/2, c_y + self.cup_ref_d/2, c_z]
        return v_min, v_max

    def detection_callback(self, msg):
        threshold = self.get_parameter('threshold').value
        marker_array = MarkerArray()

        # z값 기준으로 레이어별 그룹화 (같은 레이어 = z 차이 < cup_ref_h/2)
        z_tol = self.cup_ref_h / 2
        layers = []  # list of lists, 각 원소는 같은 레이어의 detection 목록
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

        # z 오름차순 정렬 (bottom → top)
        layers.sort(key=lambda g: g[0].bbox.center.position.z)

        max_ratio = 0.0
        for layer_idx, layer_cups in enumerate(layers):
            # 같은 레이어 내 x 오름차순 정렬
            layer_cups.sort(key=lambda d: d.bbox.center.position.x)

            for pos_i, detection in enumerate(layer_cups):
                pos = detection.bbox.center.position
                size = detection.bbox.size
                d_min = [pos.x - size.x/2, pos.y - size.y/2, pos.z - size.z]
                d_max = [pos.x + size.x/2, pos.y + size.y/2, pos.z]

                v_min, v_max = self.get_virtual_box(pos_i, layer_idx)
                ratio = self.calculate_overlap_ratio(v_min, v_max, d_min, d_max)
                max_ratio = max(max_ratio, ratio)

                occupied = ratio > threshold
                virtual_marker = self.create_marker(v_min, v_max, occupied, 300 + pos_i + layer_idx * 100)
                marker_array.markers.append(virtual_marker)

                cup_text_pos = [pos.x, pos.y, pos.z + size.z / 2]
                cup_text_marker = self.create_text_marker(cup_text_pos, f'{ratio:.2f}', 200 + pos_i + layer_idx * 100)
                marker_array.markers.append(cup_text_marker)

                det_marker = self.create_detected_cup_marker(detection, pos_i + layer_idx * 100)
                marker_array.markers.append(det_marker)

                self.pub_status.publish(Int8(data=1 if occupied else 0))
                self.get_logger().info(
                    f'Layer {layer_idx}, cup {pos_i}: '
                    f'pos ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), '
                    f'overlap: {ratio:.2f}, {"occupied" if occupied else "empty"}'
                )

        self.pub_ratio.publish(Float32(data=max_ratio))
        self.pub_marker.publish(marker_array)
        self.get_logger().info(f'Layers detected: {[len(g) for g in layers]} | Max overlap: {max_ratio:.2f}')

    def calculate_overlap_ratio(self, v_min, v_max, d_min, d_max):
        dx = max(0, min(v_max[0], d_max[0]) - max(v_min[0], d_min[0]))
        dy = max(0, min(v_max[1], d_max[1]) - max(v_min[1], d_min[1]))
        dz = max(0, min(v_max[2], d_max[2]) - max(v_min[2], d_min[2]))
        return (dx * dy * dz) / self.cup_ref_vol

def main(args=None):
    rclpy.init(args=args)
    node = CupOccupancyNode()
    rclpy.spin(node)
    rclpy.shutdown()