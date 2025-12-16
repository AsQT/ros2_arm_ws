import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class SimpleMoveNode(Node):
    def __init__(self):
        super().__init__('simple_move_node')
        
        # ==========================================
        # ĐÃ CẬP NHẬT THEO FILE SRDF CỦA BẠN
        # ==========================================
        self.group_name = "arm"            # 
        self.end_effector_link = "tcp_link" #  Dùng tcp_link để gắp chính xác
        # ==========================================

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Subscribe tọa độ từ Vision Node
        self.create_subscription(PoseStamped, '/target_grasp_pose', self.listener_callback, 10)
        
        self.target_received = False
        self.get_logger().info(f"--- WAITING FOR TARGET (Group: {self.group_name}) ---")

    def listener_callback(self, msg):
        if self.target_received: return 
        
        self.get_logger().info(f"Target Received: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}")
        
        # --- TINH CHỈNH VỊ TRÍ GẮP ---
        target_pose = msg
        # Offset Z: Đưa TCP (điểm giữa kẹp) cao hơn vật 1 chút để không đâm vào bàn
        # Vì tcp_link thường nằm ngay đầu kẹp, ta có thể để nó chạm vật hoặc cao hơn xíu
        target_pose.pose.position.z += 0.005 # +5mm so với tâm vật
        
        # (Quan trọng) Reset hướng quay của tay máy về mặc định (thẳng đứng) 
        # để tránh việc tay máy xoắn quẩy theo hướng lung tung
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 1.0 # Hướng gắp từ trên xuống (tùy robot, thử 0.707 nếu sai)
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        self.send_goal(target_pose)
        self.target_received = True

    def send_goal(self, pose_stamped):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt Server not found! Is demo.launch.py running?")
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # 1. Ràng buộc vị trí (Position Constraint)
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = self.end_effector_link # "tcp_link"
        
        # Tạo vùng chấp nhận sai số (Sphere bán kính 1cm)
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.SPHERE
        pr.dimensions = [0.01] 
        bv.primitives.append(pr)
        bv.primitive_poses.append(pose_stamped.pose)
        
        pc.constraint_region = bv
        pc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"MOVING [{self.end_effector_link}] TO TARGET...")
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()