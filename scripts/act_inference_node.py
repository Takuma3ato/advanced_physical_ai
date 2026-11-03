import os, sys
import typing, typing_extensions, enum

# --- Python 3.10 互換性パッチ群 (LeRobot 用) ---
for attr in ["Self", "Unpack", "NotRequired", "Required", "TypeAlias", "Override"]:
    if not hasattr(typing, attr) and hasattr(typing_extensions, attr):
        setattr(typing, attr, getattr(typing_extensions, attr))

if not hasattr(enum, 'StrEnum'):
    class StrEnum(str, enum.Enum):
        pass
    enum.StrEnum = StrEnum
# ------------------------------------------------

import torch, numpy as np, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64
from cv_bridge import CvBridge

home = os.path.expanduser("~")
sys.path.insert(0, f"{home}/lerobot/src")

# 未使用ポリシー (WallOSS05) のインポートエラー防止パッチ
wall_init = f"{home}/lerobot/src/lerobot/policies/wall_oss_05/__init__.py"
os.makedirs(os.path.dirname(wall_init), exist_ok=True)
with open(wall_init, "w") as f:
    f.write("class WallOSS05Policy: pass\n")

from lerobot.policies.act.modeling_act import ACTPolicy

class So100ActAutonomousNode(Node):
    def __init__(self):
        super().__init__("so100_act_autonomous_node")
        
        m_dir = f"{home}/ros2_ws/src/so100_ai_model/pretrained_model"
        # モデルのロード（正規化・逆正規化プロセッサも内部で自動適用される）
        self.policy = ACTPolicy.from_pretrained(m_dir).to("cpu").eval()
        self.policy.reset()
        
        self.bridge = CvBridge()
        
        # データ収集・学習時と完全に一致させた6関節の順序定義
        self.target_joints = [
            "shoulder_pan", 
            "shoulder_lift", 
            "elbow_flex",
            "wrist_flex", 
            "wrist_roll", 
            "gripper_joint"
        ]
        
        self.pubs = {
            name: self.create_publisher(Float64, f"/cmd_pos/{name}", 10)
            for name in self.target_joints
        }
            
        self.latest_img = None
        self.latest_state = None
        
        # --- ステートマシン管理（学習データの180フレームと完全同期） ---
        self.state = "RUNNING"
        self.ep_step = 0
        self.max_ep_steps = 180   # トリミング済み学習データ長（180フレーム）と一致
        self.reset_step = 0
        self.reset_duration = 30  # 0.6秒 (50Hz * 0.6s) でキビキビとホームへ復帰
        self.start_pos = None
        self.home_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.6], dtype=np.float32)
        
        self.create_subscription(Image, "/camera/image_raw", self.img_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.jnt_cb, 10)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("🧠 完全自律 ACT推論ノード（確定版・180step同期）が起動しました。")

    def img_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            if cv_img.shape[0] != 224 or cv_img.shape[1] != 224:
                cv_img = cv2.resize(cv_img, (224, 224), interpolation=cv2.INTER_LINEAR)
            self.latest_img = cv_img
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def jnt_cb(self, msg):
        state_dict = dict(zip(msg.name, msg.position))
        current_state = []
        
        # fixed_base等を無視し、ターゲット6関節のみを厳密な順序で取得
        for name in self.target_joints:
            if name in state_dict:
                current_state.append(state_dict[name])
            else:
                matched = [v for k, v in state_dict.items() if name in k]
                current_state.append(matched[0] if matched else 0.0)
                
        self.latest_state = np.array(current_state, dtype=np.float32)

    def control_loop(self):
        if self.latest_img is None or self.latest_state is None:
            return

        # ----------------------------------------------------
        # モード1: 初期ホーム姿勢への復帰処理 (RESETTING)
        # ----------------------------------------------------
        if self.state == "RESETTING":
            alpha = self.reset_step / float(self.reset_duration)
            target = (1.0 - alpha) * self.start_pos + alpha * self.home_pos
            
            for i, name in enumerate(self.target_joints):
                self.pubs[name].publish(Float64(data=float(target[i])))
                
            self.reset_step += 1
            if self.reset_step >= self.reset_duration:
                self.policy.reset()
                self.ep_step = 0
                self.state = "RUNNING"
                self.get_logger().info("⚡ ホーム復帰完了。遅延なく次の推論サイクルを開始します。")
            return

        # ----------------------------------------------------
        # モード2: AIによる自律推論制御 (RUNNING)
        # ----------------------------------------------------
        if self.ep_step >= self.max_ep_steps:
            self.state = "RESETTING"
            self.reset_step = 0
            self.start_pos = self.latest_state.copy()
            self.get_logger().info("🔄 エピソード完了。初期ホーム姿勢へ復帰します...")
            return

        img_tensor = torch.from_numpy(self.latest_img).permute(2, 0, 1).float() / 255.0
        state_tensor = torch.from_numpy(self.latest_state).float()
        obs_dict = {
            "observation.images.camera": img_tensor.unsqueeze(0),
            "observation.state": state_tensor.unsqueeze(0)
        }

        with torch.no_grad():
            action = self.policy.select_action(obs_dict)
        raw_cmd = action.squeeze(0).numpy()

        if self.ep_step % 20 == 0:
            self.get_logger().info(
                f"Step {self.ep_step:03d}/{self.max_ep_steps} | "
                f"Lift: {raw_cmd[1]:.3f} | Elbow: {raw_cmd[2]:.3f} | Gripper: {raw_cmd[5]:.3f}"
            )

        for i, name in enumerate(self.target_joints):
            self.pubs[name].publish(Float64(data=float(raw_cmd[i])))

        self.ep_step += 1

def main():
    rclpy.init()
    node = So100ActAutonomousNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
