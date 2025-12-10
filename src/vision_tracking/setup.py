from setuptools import setup
import os
from glob import glob

package_name = 'vision_tracking'

setup(
    name=package_name,
    version='0.0.0',
    # SỬ DỤNG PACKAGES CHUẨN
    packages=[package_name],
    
    data_files=[
        # 1. Resource Index: Dấu hiệu cho ROS 2 biết package tồn tại
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        # 2. Package Metadata
        ('share/' + package_name, ['package.xml']),
        
        # 3. Cài đặt các file Resource (Ảnh mẫu và mô hình)
        # SỬA LỖI CÚ PHÁP TẠI ĐÂY
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        
        # TÔI GIẢ ĐỊNH FILE TEST.JPG CỦA BẠN NẰM TRONG RESOURCE
        # Nếu file test.jpg nằm ở thư mục khác, bạn phải khai báo rõ ràng.
        # VÍ DỤ NẾU NÓ THỰC SỰ LÀ /home/asus/ros2_ws/src/object_detection/object_detection/test.jpg:
        # Bạn KHÔNG NÊN khai báo file từ package khác vào package này.
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='ROS 2 package for combined YOLO and SIFT object tracking.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts':[
            # Cấu trúc: [Tên_lệnh] = [tên_module].[tên_file_script]:main
            'sift_yolo_tracker_node = vision_tracking.sift_yolo_tracker_node:main',
            'yolo_publisher_node = vision_tracking.yolo_publisher_node:main',
        ],
    },
)
