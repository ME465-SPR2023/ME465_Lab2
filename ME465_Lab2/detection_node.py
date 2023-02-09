import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")
        self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.detection_callback,
            5,
        )
        self.create_subscription(
            CameraInfo,
            "/camera_info",
            self.camera_info_callback,
            5,
        )
        self.detected = self.create_publisher(
            Bool,
            '/detected',
            5,
        )
        self.declare_parameter("size", 1)
        self.size = self.get_parameter("size").value
        self.P = None
        
    def camera_info_callback(self, msg):
        self.P = msg.p.reshape((3, 4))[:, :3]
        
    def detection_callback(self, msg):
        if len(msg.detections) > 0 and self.P is not None:
            detection = msg.detections[0]
            T = np.linalg.inv(self.P) @ detection.homography.reshape((3, 3))
            tt = T[:, 2] / np.linalg.norm(T[:, 0]) * self.size / 2
            detected = Bool()
            detected.data = bool(np.abs(tt[0]) <= 0.1)
            self.detected.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
