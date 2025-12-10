#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
import time

# --- KHAI BÁO CỐ ĐỊNH ---
# Điều chỉnh đường dẫn đến file best.pt của bạn
MODEL_PATH = '/home/asus/ros2_ws/src/vision_tracking/resource/best2.pt' 
# Topic ảnh màu từ Orbbec Astra
IMAGE_SUBSCRIPTION_TOPIC = '/camera/rgb/image_raw'
# Topic xuất bản Bounding Box
DETECTION_PUBLICATION_TOPIC = '/yolo_detections' 

class YOLOPublisherNode(Node):
    def __init__(self):
        super().__init__('yolo_publisher_node')
        
        # 1. Load Mô hình YOLOv8
        try:
            self.model = YOLO(MODEL_PATH)
            self.get_logger().info(f'Đã tải thành công mô hình YOLO từ: {MODEL_PATH}')
        except Exception as e:
            self.get_logger().error(f'Lỗi khi tải mô hình YOLO: {e}')
            return

        self.bridge = CvBridge()
        
        # 2. Khởi tạo Publisher cho Bounding Box
        self.publisher_ = self.create_publisher(Pose2D, DETECTION_PUBLICATION_TOPIC, 10)
        
        # 3. Khởi tạo Subscriber cho ảnh từ Camera Orbbec
        self.subscription_img = self.create_subscription(
            Image,
            IMAGE_SUBSCRIPTION_TOPIC,
            self.image_callback,
            10
        )
        self.last_inference_time = time.time()
        # Thiết lập tốc độ chạy YOLO (ví dụ: 5 FPS, 0.2 giây/khung hình)
        self.inference_period = 0.2 
        
        self.get_logger().info('YOLO Publisher Node đã khởi động và chờ ảnh.')

    def image_callback(self, msg):
        current_time = time.time()
        # Giới hạn tốc độ chạy YOLO để tránh quá tải
        if current_time - self.last_inference_time < self.inference_period:
            return
            
        self.last_inference_time = current_time

        try:
            # Chuyển đổi ROS Image sang OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Lỗi chuyển đổi ảnh: {e}')
            return

        # 4. Chạy Inference YOLOv8
        # Sử dụng thuộc tính stream=True để xử lý frame hiệu quả hơn
        results = self.model.predict(frame, verbose=False, conf=0.5) 
        
        # Lấy kết quả từ khung hình đầu tiên (thường chỉ có 1 khung hình)
        if results and results[0].boxes:
            # Lấy Bounding Box đầu tiên được phát hiện (ví dụ: đối tượng có độ tự tin cao nhất)
            box = results[0].boxes[0]
            
            # Lấy tọa độ (x_min, y_min, x_max, y_max)
            x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy().astype(int)
            
            w = int(x_max - x_min)
            h = int(y_max - y_min)
            
            # 5. Xuất bản Bounding Box dưới dạng Pose2D (Theo quy ước tạm)
            bb_msg = Pose2D()
            bb_msg.x = float(x_min)
            bb_msg.y = float(y_min)
            bb_msg.theta = float(w) # Sử dụng theta để truyền width
            # (Node Tracker sẽ giả định height = width)

            self.publisher_.publish(bb_msg)
            self.get_logger().info(f'BB đã xuất bản: x={x_min}, y={y_min}, w={w}')

        # TÙY CHỌN: Để trực quan hóa kết quả YOLO (giúp debug)
        # annotated_frame = results[0].plot()
        # cv2.imshow("YOLO Debug", annotated_frame)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_publisher_node = YOLOPublisherNode()
    rclpy.spin(yolo_publisher_node)
    
    yolo_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()