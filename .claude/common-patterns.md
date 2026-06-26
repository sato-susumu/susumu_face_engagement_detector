# 頻用コマンドパターン

## ビルド・テスト

```bash
# 環境セットアップ
source /opt/ros/humble/setup.bash

# 通常ビルド
cd ~/ros2_ws
colcon build --packages-select susumu_face_engagement_detector
source install/setup.bash

# シンボリックリンク install (開発中、setup.py 変更時は再ビルド)
colcon build --packages-select susumu_face_engagement_detector --symlink-install

# クリーン再ビルド
rm -rf build/susumu_face_engagement_detector install/susumu_face_engagement_detector
colcon build --packages-select susumu_face_engagement_detector
```

## テスト

```bash
# Makefile 経由 (推奨)
make test              # 全テスト (unit + integration, 57件)
make test-unit         # 単体のみ

# 直接 pytest
python3 -m pytest                          # 全テスト
python3 -m pytest test/unit -v             # 単体のみ
python3 -m pytest test/integration -v      # 統合のみ
python3 -m pytest test/unit/test_engagement_scorer.py -v  # 特定ファイル
python3 -m pytest -k "yunet"               # 名前で絞り込み
```

## 評価ハーネス (outputs/ への成果物生成)

```bash
# 一発で全部 (smoke 検出 + YuNet 検出 + LFW 認識 + 可視化 + レポート)
make outputs

# 個別評価
make eval-detection-smoke       # dlib HOG 100枚 (~30秒)
make eval-detection-baseline    # dlib HOG full WIDER FACE val (~12分)
make eval-detection-yunet       # YuNet full WIDER FACE val (~1.5分)
make eval-recognition-baseline  # dlib 128-D LFW pairs test (~30秒)

# 可視化
make visualize-detection        # AP/latency 棒グラフ + オーバーレイ
make visualize-recognition      # ROC + コサイン類似度分布
make visualize-engagement       # 24秒 engagement 時系列

# レポート再生成
make eval-report                # outputs/reports/REPORT.md

# 全成果物クリア
make clean-outputs
```

## ROS 2 起動

### フルパイプライン

```bash
# YuNet + head_pose + gaze + expression + engagement
ros2 launch susumu_face_engagement_detector engagement_pipeline.launch.py
```

### 個別ノード

```bash
# 検出 (YuNet)
ros2 run susumu_face_engagement_detector face_detection_node \
    --ros-args -p detection_backend:=yunet \
               -p model_path:=$HOME/models/face_detection/face_detection_yunet_2023mar.onnx

# 検出 (dlib HOG fallback)
ros2 run susumu_face_engagement_detector face_detection_node

# 認識・姿勢・視線・表情・engagement
ros2 run susumu_face_engagement_detector face_recognition_node
ros2 run susumu_face_engagement_detector head_pose_node
ros2 run susumu_face_engagement_detector gaze_node
ros2 run susumu_face_engagement_detector expression_node
ros2 run susumu_face_engagement_detector engagement_node
```

### 個別 launch

```bash
ros2 launch susumu_face_engagement_detector face_detection_only.launch.py
ros2 launch susumu_face_engagement_detector face_recognition_only.launch.py
ros2 launch susumu_face_engagement_detector face_detection_rviz.launch.py
```

## トピック確認

```bash
# 標準 (推奨) ROS4HRI トピック
ros2 topic echo /face_detections_vision               # vision_msgs/Detection2DArray
ros2 topic echo /humans/faces/tracked                 # hri_msgs/IdsList
ros2 topic echo /humans/faces/head_pose               # geometry_msgs/PoseStamped
ros2 topic echo /humans/faces/gaze                    # geometry_msgs/Vector3Stamped
ros2 topic echo /humans/faces/expression              # hri_msgs/Expression
ros2 topic echo /humans/persons/default/engagement_status   # hri_msgs/EngagementLevel
ros2 topic echo /humans/persons/default/engagement_score    # std_msgs/Float32

# 旧 String I/F (後方互換)
ros2 topic echo /face_detections    # std_msgs/String パイプ区切り
ros2 topic echo /face_identities
ros2 topic echo /engagement_event   # "<person_id>:ENGAGED" 等

# 統計
ros2 topic hz /face_detections_vision
ros2 topic bw /face_detections_vision
ros2 topic delay /face_detections_vision
```

## パラメータ確認・変更

```bash
# 一覧
ros2 param list /face_detection_node
ros2 param list /engagement_node

# 取得
ros2 param get /face_detection_node detection_backend
ros2 param get /engagement_node engaged_threshold

# 設定 (型に注意)
ros2 param set /face_detection_node downsample_factor 1.0
ros2 param set /engagement_node engaged_threshold 0.7
```

## 評価データ取得

```bash
# WIDER FACE val (363 MB、HuggingFace ミラー)
mkdir -p ~/datasets/wider_face && cd ~/datasets/wider_face
wget https://huggingface.co/datasets/CUHK-CSE/wider_face/resolve/main/data/wider_face_split.zip
wget https://huggingface.co/datasets/CUHK-CSE/wider_face/resolve/main/data/WIDER_val.zip
unzip wider_face_split.zip
unzip WIDER_val.zip

# YuNet 重み (Apache 2.0)
mkdir -p ~/models/face_detection
wget https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx \
    -O ~/models/face_detection/face_detection_yunet_2023mar.onnx

# HSEmotion 重み (Apache 2.0)
mkdir -p ~/.hsemotion
wget https://github.com/HSE-asavchenko/face-emotion-recognition/raw/main/models/affectnet_emotions/onnx/enet_b0_8_best_afew.onnx \
    -O ~/.hsemotion/enet_b0_8_best_afew.onnx
```

## デバッグ

```bash
# ノードグラフ可視化
rqt_graph

# ロギング閾値変更
ros2 run susumu_face_engagement_detector face_detection_node --ros-args --log-level debug

# 特定ノードを冷ます (発行を一時停止)
ros2 lifecycle set /face_detection_node deactivate    # ※ ライフサイクル非対応の場合は ctrl-C で

# トピック未配信の調査
ros2 topic info /humans/faces/gaze
ros2 node info /gaze_node
```

## rosbag2

```bash
# 録画
ros2 bag record -s mcap /camera/color/image_raw -o /tmp/session.bag

# 再生
ros2 bag play /tmp/session.bag

# 評価用に画像列を bag 化 (eval/rosbag_converters/)
python3 -m eval.rosbag_converters.images_to_rosbag \
    --wider-val-root ~/datasets/wider_face \
    --output /tmp/wider_val.bag \
    --fps 5.0 \
    --limit 100
```

## 整理・クリーンアップ

```bash
make clean-outputs           # outputs/ の生成物のみ削除 (README/.gitkeep/.gitignore は残す)
find . -name __pycache__ -exec rm -rf {} +  # Python キャッシュ
rm -rf build install log     # ROS 2 ビルド成果物
```
