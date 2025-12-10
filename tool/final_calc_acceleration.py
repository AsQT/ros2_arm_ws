import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import InteractiveMarkerFeedback
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
import tkinter as tk
import threading
import math
import time
import copy

# --- BIẾN TOÀN CỤC ---
SHARED_POSE = None        
SHARED_FRAME = 'world'
SHARED_RESULT = None      
GUI_STATUS = "Đang khởi động..."
GUI_COLOR = "blue"

# ==========================================
# NODE 1: TAI NGHE
# ==========================================
class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node_acc')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(InteractiveMarkerFeedback, 
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
            self.callback, qos)

    def callback(self, msg):
        global SHARED_POSE, SHARED_FRAME, GUI_STATUS, GUI_COLOR
        SHARED_POSE = msg.pose
        SHARED_FRAME = msg.header.frame_id
        GUI_STATUS = f"Frame: {msg.header.frame_id}"
        GUI_COLOR = "orange"

# ==========================================
# NODE 2: TAY GỌI
# ==========================================
class WorkerNode(Node):
    def __init__(self):
        super().__init__('worker_node_acc')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')

# ==========================================
# LUỒNG XỬ LÝ LOGIC (TÍNH GIA TỐC)
# ==========================================
def processing_thread(worker_node):
    global SHARED_POSE, SHARED_FRAME, SHARED_RESULT, GUI_STATUS, GUI_COLOR
    
    # Biến lưu trạng thái cũ để tính đạo hàm
    prev_positions = {}   # Lưu góc cũ
    prev_velocities = {}  # Lưu vận tốc cũ
    prev_time = time.time()

    # Đợi service
    while not worker_node.cli.service_is_ready():
        time.sleep(1)
        print("Đang đợi service IK...")
    print("Worker đã sẵn sàng!")

    while rclpy.ok():
        if SHARED_POSE is None:
            time.sleep(0.02)
            continue
            
        current_pose = copy.deepcopy(SHARED_POSE)
        current_frame = copy.deepcopy(SHARED_FRAME)
        
        # --- GỌI IK ---
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm' 
        req.ik_request.avoid_collisions = False
        
        ps = PoseStamped()
        ps.header.frame_id = current_frame
        ps.header.stamp.sec = 0; ps.header.stamp.nanosec = 0
        ps.pose = current_pose
        req.ik_request.pose_stamped = ps
        
        try:
            future = worker_node.cli.call_async(req)
            while not future.done() and rclpy.ok():
                time.sleep(0.005) # Ngủ cực ngắn để lấy mẫu nhanh hơn
            
            if future.done():
                res = future.result()
                if res.error_code.val == 1:
                    GUI_STATUS = "SUCCESS"
                    GUI_COLOR = "green"
                    
                    current_time = time.time()
                    dt = current_time - prev_time
                    if dt == 0: dt = 0.001 # Tránh chia cho 0
                    
                    data_package = {}
                    js = res.solution.joint_state
                    
                    for i, name in enumerate(js.name):
                        if len(js.position) > i:
                            # 1. Lấy vị trí mới (Độ)
                            pos_deg = math.degrees(js.position[i])
                            
                            # 2. Tính Vận tốc (Delta Pos / dt)
                            old_pos = prev_positions.get(name, pos_deg)
                            velocity = (pos_deg - old_pos) / dt
                            
                            # 3. Tính Gia tốc (Delta Vel / dt)
                            old_vel = prev_velocities.get(name, 0.0)
                            acceleration = (velocity - old_vel) / dt
                            
                            # Lưu lại cho vòng sau
                            prev_positions[name] = pos_deg
                            prev_velocities[name] = velocity
                            
                            # Đóng gói dữ liệu
                            data_package[name] = {
                                'pos': pos_deg,
                                'acc': acceleration
                            }
                    
                    prev_time = current_time
                    SHARED_RESULT = data_package
                else:
                    GUI_STATUS = f"Lỗi IK: {res.error_code.val}"
                    GUI_COLOR = "red"
        except Exception as e:
            print(f"Lỗi: {e}")
            
        time.sleep(0.02) # Tần suất lấy mẫu (~50Hz)

# ==========================================
# GUI APP
# ==========================================
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("IK & ACCELERATION MONITOR")
        self.root.geometry("500x550")
        
        self.lbl_stt = tk.Label(root, text="...", fg="blue", font=("Arial", 11, "bold"))
        self.lbl_stt.pack(pady=10)
        
        # Header
        f_head = tk.Frame(root)
        f_head.pack(fill="x", padx=20)
        tk.Label(f_head, text="Joint Name", width=12, anchor="w").pack(side="left")
        tk.Label(f_head, text="Position (°)", width=10, fg="purple").pack(side="left")
        tk.Label(f_head, text="Accel (°/s²)", width=12, fg="red").pack(side="right")

        self.con = tk.Frame(root)
        self.con.pack(fill="both", expand=True, padx=20)
        self.rows = {}
        
        self.update_ui()

    def update_ui(self):
        self.lbl_stt.config(text=GUI_STATUS, fg=GUI_COLOR)
        
        if SHARED_RESULT:
            for name, data in SHARED_RESULT.items():
                pos = data['pos']
                acc = data['acc']
                
                if name not in self.rows:
                    f = tk.Frame(self.con)
                    f.pack(fill="x", pady=5)
                    
                    # Tên khớp
                    tk.Label(f, text=name, width=12, anchor="w", font=("bold")).pack(side="left")
                    
                    # Label Góc (Tím)
                    lbl_pos = tk.Label(f, text="--", width=8, fg="purple", font=("Arial", 10, "bold"))
                    lbl_pos.pack(side="left")
                    
                    # Slider
                    sl = tk.Scale(f, from_=-180, to=180, orient="horizontal", showvalue=0, length=100)
                    sl.pack(side="left", padx=5)
                    
                    # Label Gia tốc (Đỏ)
                    lbl_acc = tk.Label(f, text="--", width=10, fg="red", anchor="e")
                    lbl_acc.pack(side="right")
                    
                    self.rows[name] = (lbl_pos, sl, lbl_acc)
                
                lbl_p, slider, lbl_a = self.rows[name]
                lbl_p.config(text=f"{pos:.1f}")
                slider.set(pos)
                
                # Hiển thị gia tốc (làm tròn số nguyên cho đỡ rối)
                lbl_a.config(text=f"{acc:.0f}")
        
        self.root.after(50, self.update_ui) # Cập nhật nhanh hơn

def main():
    rclpy.init()
    listener = ListenerNode()
    worker = WorkerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(worker)
    
    t_ros = threading.Thread(target=executor.spin, daemon=True)
    t_ros.start()
    
    t_logic = threading.Thread(target=processing_thread, args=(worker,), daemon=True)
    t_logic.start()
    
    root = tk.Tk()
    App(root)
    root.mainloop()
    
    executor.shutdown()
    listener.destroy_node()
    worker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
