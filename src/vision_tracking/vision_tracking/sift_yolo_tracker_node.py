#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D 
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
# from ament_index_python.packages import get_package_share_directory # Không dùng vì nó gây lỗi nếu package chưa setup đầy đủ

# --- KHAI BÁO CỐ ĐỊNH ---
# Giả định đường dẫn ảnh mẫu, bạn nên đặt file Book_2.jpg cạnh file python này
TEMPLATE_PATH = 'Book_2.jpg' 

# TÊN TOPIC ẢNH TỪ CAMERA ORBBEC
# Topic chuẩn cho ảnh màu RGB của Orbbec Astra.
ORBBEC_IMAGE_TOPIC = '/camera/rgb/image_raw' # HOẶC /camera/color/image_raw

class SIFT_YOLO_TrackerNode(Node):
    def __init__(self):
        super().__init__('sift_yolo_tracker_node')
        
        # --- Khởi tạo ROS 2 ---
        self.bridge = CvBridge()
        
        # LẮNG NGHE LUỒNG ẢNH TỪ ORBBEC ASTRA
        self.get_logger().info(f'Đang lắng nghe ảnh từ topic: {ORBBEC_IMAGE_TOPIC}')
        self.subscription_img = self.create_subscription(
            Image,
            ORBBEC_IMAGE_TOPIC,  # ĐÃ THAY ĐỔI
            self.image_callback,
            10
        )
        
        # Lắng nghe Bounding Box từ Node YOLO (Vẫn dùng Pose2D như quy ước tạm)
        self.subscription_yolo = self.create_subscription(
            Pose2D, 
            '/yolo_detections',
            self.yolo_detection_callback,
            10
        )
        
        # Xuất bản ảnh kết quả
        self.publisher_ = self.create_publisher(Image, '/tracking_result', 10)
        
        # --- Khởi tạo Logic Tracking ---
        self.sift = cv2.SIFT_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        self.trajectory = []
        
        # Trạng thái tracking
        self.is_tracking_initialized = False
        self.last_frame = None
        self.current_frame = None
        
        # Descriptor và Kích thước của Mẫu SIFT HIỆN TẠI (sẽ được cập nhật từ YOLO)
        self.des_template = None
        self.kp_template = None
        self.w_temp = 0
        self.h_temp = 0
        
        # Tải ảnh mẫu ban đầu để khởi tạo SIFT nếu cần
        self.initialize_sift_with_default_template()
        
        self.get_logger().info('SIFT-YOLO Tracker Node đã khởi động, chờ dữ liệu từ Orbbec và YOLO.')

    def initialize_sift_with_default_template(self):
        """Tải ảnh mẫu ban đầu (Book_2.jpg) để có descriptor dự phòng."""
        try:
            img_template = cv2.imread(TEMPLATE_PATH)
            if img_template is None:
                self.get_logger().warn(f"Không tìm thấy ảnh mẫu tại: {TEMPLATE_PATH}. SIFT sẽ chỉ khởi tạo khi YOLO phát hiện đối tượng.")
                return

            gray_template = cv2.cvtColor(img_template, cv2.COLOR_BGR2GRAY)
            self.kp_template, self.des_template = self.sift.detectAndCompute(gray_template, None)
            self.h_temp, self.w_temp = gray_template.shape
            
            if self.des_template is not None and len(self.des_template) > 10:
                self.is_tracking_initialized = True
                self.get_logger().info('SIFT initialized with default template.')
            else:
                 self.get_logger().warn('Ảnh mẫu có nhưng không đủ SIFT keypoint, chờ YOLO.')
            
        except Exception as e:
            self.get_logger().error(f"Lỗi khởi tạo SIFT ban đầu: {e}")

    def yolo_detection_callback(self, msg):
        """
        Hàm được gọi khi YOLO phát hiện đối tượng. 
        Dùng Bounding Box của YOLO để cắt và cập nhật SIFT Template.
        """
        if self.current_frame is None:
            self.get_logger().warn("Nhận được Bounding Box nhưng chưa có khung hình ảnh từ Orbbec.")
            return

        # QUY ƯỚC TẠM (NHƯ ĐÃ THỐNG NHẤT): x, y là góc trên trái; theta là width. Height = Width.
        x = int(msg.x)
        y = int(msg.y)
        w = int(msg.theta)
        h = w # <--- Giả định chiều cao bằng chiều rộng

        # Đảm bảo Bounding Box hợp lệ và nằm trong khung hình
        x = max(0, x)
        y = max(0, y)
        w = min(self.current_frame.shape[1] - x, w)
        h = min(self.current_frame.shape[0] - y, h)

        if w > 10 and h > 10:
            # Cắt ảnh theo Bounding Box của YOLO
            roi = self.current_frame[y:y+h, x:x+w]
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Cập nhật SIFT Template
            self.kp_template, self.des_template = self.sift.detectAndCompute(gray_roi, None)
            self.w_temp, self.h_temp = roi.shape[1], roi.shape[0]
            
            if self.des_template is not None and len(self.des_template) > 10:
                self.is_tracking_initialized = True
                self.get_logger().info(f'SIFT Template được cập nhật từ YOLO. Kích thước: {self.w_temp}x{self.h_temp}')
                self.trajectory.clear()
            else:
                self.get_logger().warn("YOLO phát hiện nhưng không đủ SIFT Keypoint, không khởi tạo tracking.")


    def image_callback(self, msg):
        """Hàm callback chính để xử lý và theo dõi ảnh."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_frame = frame.copy() 
            
            if not self.is_tracking_initialized or self.des_template is None:
                cv2.putText(frame, "CHO PHAT HIEN YOLO...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.publish_result(frame)
                return

            # --- SIFT Tracking Logic ---
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_frame = cv2.equalizeHist(gray_frame)
            kp_frame, des_frame = self.sift.detectAndCompute(gray_frame, None)

            if des_frame is None or len(des_frame) < 10:
                cv2.putText(frame, "KHONG DU KEYPOINT SIFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.publish_result(frame)
                return

            # So khớp và tính toán Homography
            matches = self.bf.match(self.des_template, des_frame)
            matches = sorted(matches, key=lambda x: x.distance)[:70]

            if len(matches) > 10: 
                src_pts = np.float32([self.kp_template[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                # --- Vẽ kết quả Homography ---
                if M is not None:
                    pts = np.float32([[0, 0], [self.w_temp, 0],
                                      [self.w_temp, self.h_temp],
                                      [0, self.h_temp]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, M)
                    cv2.polylines(frame, [np.int32(dst)], True, (0, 255, 0), 3) # Màu XANH LÁ
                    
                    # Cập nhật và vẽ quỹ đạo
                    center = np.mean(dst, axis=0)[0]
                    self.trajectory.append(center)
                    
                    for i in range(1, len(self.trajectory)):
                        pt1 = tuple(np.int32(self.trajectory[i-1]))
                        pt2 = tuple(np.int32(self.trajectory[i]))
                        cv2.line(frame, pt1, pt2, (0, 0, 255), 2)
                    
                    if len(self.trajectory) > 20: 
                        self.trajectory = self.trajectory[-20:]
                
                else:
                    cv2.putText(frame, "SIFT TRACKING MAT (Khong tim thay Homography)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    self.is_tracking_initialized = False 
            
            else:
                cv2.putText(frame, "SO KHOP YEU - KHOI TAO LAI", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.is_tracking_initialized = False 
            
            self.publish_result(frame)
            
        except Exception as e:
            self.get_logger().error(f'Lỗi trong callback ảnh: {e}')

    def publish_result(self, frame):
        """Hàm đóng gói và xuất bản ảnh kết quả."""
        tracking_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(tracking_msg)

def main(args=None):
    rclpy.init(args=args)
    sift_yolo_tracker_node = SIFT_YOLO_TrackerNode()
    rclpy.spin(sift_yolo_tracker_node)
    
    sift_yolo_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
