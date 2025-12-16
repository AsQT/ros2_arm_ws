import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RedObjectPicker(Node):
    def __init__(self):
        super().__init__('red_object_picker')

        # --- CẤU HÌNH ---
        # Tên frame của chân robot (gốc tọa độ)
        self.base_frame = "base_link" 
        # Tên frame của camera (Orbbec Astra thường là tên này, nếu lỗi hãy check RViz)
        self.camera_frame = "camera_color_optical_frame" 
        
        # --- KHỞI TẠO ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher: Gửi vị trí gắp cho MoveIt hoặc Controller
        self.target_pub = self.create_publisher(PoseStamped, '/target_grasp_pose', 10)

        # Subscriber: Đăng ký nhận ảnh màu và độ sâu
        # LƯU Ý: Nếu topic camera của bạn khác, hãy sửa ở đây (dùng 'ros2 topic list' để xem)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        self.cv_image = None
        self.depth_image = None
        self.camera_model = None
        
        # Timer: Chạy vòng lặp xử lý 1 lần/giây để tránh lag
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info("--- READY TO PICK RED OBJECT ---")

    def info_callback(self, msg):
        if self.camera_model is None:
            self.camera_model = msg
            self.get_logger().info("Camera Info Received")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error (Color): {e}")

    def depth_callback(self, msg):
        try:
            # Ảnh depth thường là 16UC1 (mm) hoặc 32FC1 (m)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error (Depth): {e}")

    def get_red_center(self):
        """Tìm tâm vật thể màu đỏ trong ảnh"""
        if self.cv_image is None: return None

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Dải màu đỏ 1 (0-10)
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        # Dải màu đỏ 2 (160-180)
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 + mask2

        # Khử nhiễu
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Tìm contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Lấy vật to nhất
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 400: # Chỉ nhận vật đủ lớn
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
        return None

    def control_loop(self):
        """Hàm chính: Nhìn -> Tính toán -> Ra lệnh"""
        if self.cv_image is None or self.depth_image is None or self.camera_model is None:
            self.get_logger().warn("Waiting for camera data...", throttle_duration_sec=2)
            return

        # 1. Tìm tâm pixel (u, v)
        center = self.get_red_center()
        if center:
            u, v = center
            
            # 2. Lấy độ sâu Z (khoảng cách)
            try:
                raw_depth = self.depth_image[v, u]
                if raw_depth == 0: 
                    self.get_logger().warn("Depth is 0 at center, skipping...")
                    return
                
                # Orbbec thường trả mm, chuyển sang mét
                z_metric = raw_depth / 1000.0 
            except IndexError:
                return

            # 3. Tính tọa độ 3D so với Camera (X_cam, Y_cam, Z_cam)
            # FX, FY, CX, CY từ ma trận K
            fx = self.camera_model.k[0]
            fy = self.camera_model.k[4]
            cx = self.camera_model.k[2]
            cy = self.camera_model.k[5]

            x_cam = (u - cx) * z_metric / fx
            y_cam = (v - cy) * z_metric / fy
            z_cam = z_metric

            # 4. Transform: Chuyển từ Camera Frame -> Base Frame (Robot)
            pose_cam = PoseStamped()
            pose_cam.header.frame_id = self.camera_frame
            pose_cam.header.stamp = self.get_clock().now().to_msg()
            pose_cam.pose.position.x = x_cam
            pose_cam.pose.position.y = y_cam
            pose_cam.pose.position.z = z_cam
            pose_cam.pose.orientation.w = 1.0

            try:
                # Tìm transform mới nhất
                if not self.tf_buffer.can_transform(self.base_frame, self.camera_frame, rclpy.time.Time()):
                    self.get_logger().warn("Waiting for TF transform...")
                    return

                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, rclpy.time.Time())
                
                pose_base = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)

                # --- KẾT QUẢ ---
                x_real = pose_base.pose.position.x
                y_real = pose_base.pose.position.y
                z_real = pose_base.pose.position.z

                self.get_logger().info(
                    f"\n>>> FOUND RED OBJECT! <<<\n"
                    f"Pixel: ({u}, {v}) | Depth: {z_metric:.2f}m\n"
                    f"Robot Base Coordinates:\n"
                    f" X: {x_real:.3f}\n"
                    f" Y: {y_real:.3f}\n"
                    f" Z: {z_real:.3f}"
                )

                # Publish tọa độ để MoveIt thực hiện
                self.target_pub.publish(pose_base)

            except Exception as e:
                self.get_logger().error(f"Transform Error: {e}")
        else:
            self.get_logger().info("Searching for red object...", throttle_duration_sec=2)

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
