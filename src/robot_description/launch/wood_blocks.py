#!/usr/bin/env python3
import os
import random
import subprocess
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class RandomSpawner(Node):
    def __init__(self):
        super().__init__('random_wood_spawner')

        self.declare_parameter('count', 1)
        self.declare_parameter('seed', 0)
        self.declare_parameter('x_min', 0.35)
        self.declare_parameter('x_max', 0.65)
        self.declare_parameter('y_min', -0.20)
        self.declare_parameter('y_max', 0.20)
        self.declare_parameter('z', 1.15) 

        pkg_share =get_package_share_directory('robot_description')
        sdf_path  =os.path.join(pkg_share, 'launch', 'wood_model.sdf')
        
        if not os.path.exists(sdf_path):
            self.get_logger().error(f"LOI: Khong tim thay file SDF tai: {sdf_path}")
            return

        with open(sdf_path, 'r') as f:
            self.sdf_xml =f.read()

        time.sleep(5.0) 
        self.spawn_random_objects()

    def spawn_random_objects(self):
        count =int(self.get_parameter('count').value)
        seed  =int(self.get_parameter('seed').value)
        
        if seed != 0: random.seed(seed)

        x_min =float(self.get_parameter('x_min').value)
        x_max =float(self.get_parameter('x_max').value)
        y_min =float(self.get_parameter('y_min').value)
        y_max =float(self.get_parameter('y_max').value)
        z     =float(self.get_parameter('z').value)

        self.get_logger().info(f"--- Bat dau tao {count} khoi go ---")

        for i in range(count):
            name =f"wood_{i+1}"
            x    =random.uniform(x_min, x_max)
            y    =random.uniform(y_min, y_max)
            
            self.spawn_via_command(name, x, y, z)
            time.sleep(0.5) 

    def spawn_via_command(self, name, x, y, z):
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-string', self.sdf_xml,
            '-allow_renaming', 'true'
        ]
        
        self.get_logger().info(f"Dang goi lenh spawn: {name} tai [{x:.2f}, {y:.2f}]")
        
        try:
            result =subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"-> OK: {name}")
            else:
                self.get_logger().error(f"-> LOI: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main():
    rclpy.init()
    node =RandomSpawner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()