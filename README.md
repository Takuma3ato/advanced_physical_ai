## リポジトリ構成

```
.
├── simulation/
│   ├── table_obstacle.sdf          # 障害物を配置したGazeboワールドファイル
│   └── sim_bringup.launch.py       # GazeboとROS 2ブリッジを一括起動するLaunchファイル
├── scripts/
│   ├── generate_demo_trajectory.py # 障害物回避デモ軌道生成スクリプト
│   ├── collect_episodes.sh         # rosbag2データ収集自動化シェル
│   ├── convert_dataset.py          # カメラ同期・モーショントリミング・関節辞書マッピング対応変換スクリプト
│   └── act_inference_node.py       # 完全自律ACT推論ノード（ステートマシン・CvBridge対応確定版）
└── training/
    └── so100_act_training.ipynb    # Google Colabで実行した再学習用ノートブック（.ipynb）
```
