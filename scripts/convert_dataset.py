import os, glob, shutil
import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState

TARGET_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper_joint"
]

def process_bag(bag_path, out_dir, max_frames=180):
    os.makedirs(f"{out_dir}/images", exist_ok=True)
    bridge = CvBridge()
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    
    latest_state = None
    synced_images = []
    synced_states = []
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if topic == '/joint_states':
            msg = deserialize_message(data, JointState)
            state_dict = dict(zip(msg.name, msg.position))
            state_arr = []
            for j_name in TARGET_JOINTS:
                if j_name in state_dict:
                    state_arr.append(state_dict[j_name])
                else:
                    matched = [v for k, v in state_dict.items() if j_name in k]
                    state_arr.append(matched[0] if matched else 0.0)
            latest_state = state_arr
            
        elif topic == '/camera/image_raw':
            # 関節状態が最低1回受信された後のカメラフレームのみを1対1で同期記録
            if latest_state is not None:
                msg = deserialize_message(data, Image)
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img = cv2.resize(img, (224, 224))
                synced_images.append(img)
                synced_states.append(list(latest_state))
                
    if not synced_states:
        return

    states_np = np.array(synced_states, dtype=np.float32)
    
    # --- 【自動モーショントリミング】最初の待機（sleep）区間をカット ---
    diff = np.max(np.abs(states_np - states_np[0]), axis=1)
    motion_indices = np.where(diff > 0.015)[0]
    
    if len(motion_indices) > 0:
        start_idx = max(0, motion_indices[0] - 3)
    else:
        start_idx = 0
        
    end_idx = min(len(states_np), start_idx + max_frames)
    trimmed_states = states_np[start_idx:end_idx]
    trimmed_images = synced_images[start_idx:end_idx]
    
    actual_len = len(trimmed_states)
    if actual_len < max_frames and actual_len > 0:
        pad_len = max_frames - actual_len
        trimmed_states = np.pad(trimmed_states, ((0, pad_len), (0, 0)), mode='edge')
        last_img = trimmed_images[-1]
        for _ in range(pad_len):
            trimmed_images.append(last_img)
            
    for idx, img in enumerate(trimmed_images):
        cv2.imwrite(f"{out_dir}/images/frame_{idx:05d}.jpg", img)
        
    actions_np = np.roll(trimmed_states, -2, axis=0)
    actions_np[-2:] = trimmed_states[-1]
    
    np.save(f"{out_dir}/states.npy", trimmed_states)
    np.save(f"{out_dir}/actions.npy", actions_np)
    print(f"Processed {os.path.basename(bag_path)}: raw_synced={len(states_np)}, start_idx={start_idx}, saved={len(trimmed_states)} frames.")

shutil.rmtree('/home/ubuntu/ros2_ws/data/processed', ignore_errors=True)
bag_dirs = sorted(glob.glob('/home/ubuntu/ros2_ws/data/bags/episode_*'))
for bag in bag_dirs:
    ep_name = os.path.basename(bag)
    process_bag(bag, f"/home/ubuntu/ros2_ws/data/processed/{ep_name}")
print("✅ カメラ同期＆モーショントリミング変換が完了しました！")
