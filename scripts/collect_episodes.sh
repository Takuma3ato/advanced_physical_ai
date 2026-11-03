#!/bin/bash
rm -rf /home/ubuntu/ros2_ws/data/bags
rm -rf /home/ubuntu/ros2_ws/data/processed
mkdir -p /home/ubuntu/ros2_ws/data/bags

for i in $(seq -w 1 100); do
  echo "=== Recording Episode $i / 100 ==="
  ros2 bag record --use-sim-time /camera/image_raw /joint_states -o /home/ubuntu/ros2_ws/data/bags/episode_$i &
  BAG_PID=$!
  sleep 2
  
  # デモ軌道コマンドの送信
  python3 /home/ubuntu/ros2_ws/scripts/generate_demo_trajectory.py
  
  # Gazebo上の物理的な動きが完了するまで待機（最適化: 15秒）
  echo "Waiting for Gazebo physics to complete (15s)..."
  sleep 15
  
  kill -INT $BAG_PID
  wait $BAG_PID 2>/dev/null || true
  echo "Episode $i finished."
  sleep 1
done
