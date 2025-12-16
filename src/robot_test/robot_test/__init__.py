def __init__(self):
        super().__init__('red_object_picker')

        # === CẤU HÌNH CHO GAZEBO (Sửa đoạn này) ===
        # Gazebo thường không có chữ "color" trong topic
        self.color_topic = '/camera/image'       
        self.depth_topic = '/camera/depth/image'
        self.info_topic = '/camera/camera_info'
        
        # Quan trọng: Frame trong Gazebo thường là "camera_link_optical" hoặc "camera_link"
        # Bạn nên mở RViz lên, tắt hết các tick, bật "TF" để xem cái trục camera tên chính xác là gì
        self.camera_frame = "camera_link_optical" 
        self.base_frame = "base_link" 
        
        # --- KHỞI TẠO (Giữ nguyên) ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher: Gửi vị trí gắp
        self.target_pub = self.create_publisher(PoseStamped, '/target_grasp_pose', 10)

        # Subscriber: Đăng ký nhận ảnh (Sửa lại để dùng biến self.color_topic...)
        self.create_subscription(Image, self.color_topic, self.image_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.info_callback, 10)

        self.cv_image = None
        self.depth_image = None
        self.camera_model = None
        
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info("--- READY TO PICK RED OBJECT IN GAZEBO ---")
