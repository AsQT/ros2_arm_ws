import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('/home/asus/ros2_ws/src/object_detection/object_detection/best1.pt')

        # Subscribe RGB + Depth + Calibration
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.caminfo_callback, 10)

        self.camera_matrix = None
        self.depth_image = None
        self.get_logger().info(' Detector node started, waiting for camera...')

    def caminfo_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.camera_matrix is None or self.depth_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        annotated = results[0].plot()

        # Lấy kết quả YOLO
        for box in results[0].boxes.xyxy:
            x1, y1, x2, y2 = map(int, box)
            cx, cy = (x1 + x2)//2, (y1 + y2)//2
            depth = float(self.depth_image[cy, cx]) / 1000.0  # mm → m

            if self.camera_matrix is not None:
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx_c = self.camera_matrix[0, 2]
                cy_c = self.camera_matrix[1, 2]

                X = (cx - cx_c) * depth / fx
                Y = (cy - cy_c) * depth / fy
                Z = depth

                text = f"({X:.2f}, {Y:.2f}, {Z:.2f}) m"
                cv2.putText(annotated, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("YOLO + Depth", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
