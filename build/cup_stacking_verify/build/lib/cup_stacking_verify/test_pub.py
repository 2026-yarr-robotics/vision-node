import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Vector3

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(Detection3DArray, '/detected_cups', 10)
        self.timer = self.create_timer(1.0, self.publish_test_data)

    def publish_test_data(self):
        msg = Detection3DArray()

        CUP_W = 0.078
        CUP_H = 0.095
        LAYER_H = CUP_H + 0.02   # 0.115, matches verifier's layer_height
        X0 = 0.5
        Z0 = 0.1                  # z = top of cup (verifier convention)

        # 1단(bottom): 3개 — x 간격 CUP_W
        for i in range(3):
            detection = Detection3D()
            detection.bbox.center.position.x = X0 + i * CUP_W
            detection.bbox.center.position.y = 0.0
            detection.bbox.center.position.z = Z0
            detection.bbox.size.x = CUP_W
            detection.bbox.size.y = CUP_W
            detection.bbox.size.z = CUP_H
            msg.detections.append(detection)

        # 2단(middle): 2개 — 하단 컵 사이에 위치 (x offset = CUP_W/2)
        for i in range(2):
            detection = Detection3D()
            detection.bbox.center.position.x = X0 + CUP_W / 2 + i * CUP_W
            detection.bbox.center.position.y = 0.0
            detection.bbox.center.position.z = Z0 + LAYER_H
            detection.bbox.size.x = CUP_W
            detection.bbox.size.y = CUP_W
            detection.bbox.size.z = CUP_H
            msg.detections.append(detection)

        # 3단(top): 1개 — 중간 컵 사이 중앙
        detection = Detection3D()
        detection.bbox.center.position.x = X0 + CUP_W
        detection.bbox.center.position.y = 0.0
        detection.bbox.center.position.z = Z0 + 2 * LAYER_H
        detection.bbox.size.x = CUP_W
        detection.bbox.size.y = CUP_W
        detection.bbox.size.z = CUP_H
        msg.detections.append(detection)

        self.publisher.publish(msg)
        self.get_logger().info('Published 3-2-1 pyramid test data.')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()