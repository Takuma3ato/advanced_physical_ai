import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class DemoTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('demo_trajectory_generator')
        self.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper_joint']
        self.pubs = [self.create_publisher(Float64, f'/cmd_pos/{j}', 10) for j in self.joint_names]

    def run_episode(self):
        waypoints = [
            [0.0, 0.0,   0.0,   0.0,  0.0,  0.6], # 1: 待機（全開）
            [0.0, 0.75, -0.75,  0.0,  0.0,  0.6], # 2: 障害物を高く乗り越える
            [0.0, 1.05, -1.25,  0.25, 0.0,  0.6], # 3: キューブへ深く伸ばして接地
            [0.0, 1.05, -1.25,  0.25, 0.0, -0.1], # 4: キューブをしっかり掴む（全閉）
            [0.0, 0.60, -0.80, -0.10, 0.0, -0.1], # 5: キューブを持ったまま障害物を越えて持ち上げ
            [0.0, 0.0,   0.0,   0.0,  0.0, -0.1], # 6: ホーム位置へ回収完了
        ]
        current_pos = np.zeros(6)
        
        for step_idx, pt in enumerate(waypoints):
            target_pos = np.array(pt, dtype=float)
            noise = 0.03 if step_idx in [1, 2, 4] else 0.01
            for i in range(5):
                target_pos[i] += np.random.uniform(-noise, noise)
                
            pos_trajectory = np.linspace(current_pos, target_pos, 35)
            for pos in pos_trajectory:
                for pub, val in zip(self.pubs, pos):
                    pub.publish(Float64(data=float(val)))
                time.sleep(0.02)
            current_pos = target_pos
            time.sleep(0.08)
        self.get_logger().info("✅ 把持デモ軌道を送信しました")

def main():
    rclpy.init()
    node = DemoTrajectoryGenerator()
    time.sleep(0.2)
    node.run_episode()
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
