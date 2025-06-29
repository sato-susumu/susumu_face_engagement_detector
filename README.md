# Susumu Face Engagement Detector

⚠️ **開発中のプロジェクトです** ⚠️

このプロジェクトは現在開発中です。機能の追加や変更が頻繁に行われる可能性があります。

---

顔検出・認識・注視判定を行うROS2パッケージです。効率的な動作のため、複数のノードに分割して実装されています。

## 構成

### ノード構成
- **face_detection_node**: 画像から顔を検出し、顔エンコーディングを生成
- **face_recognition_node**: 顔エンコーディングから人物を識別
- **gaze_analysis_node**: 顔の位置から注視状態を判定
- **engagement_manager_node**: 全体の状態管理とイベント発行

### トピック構成
```
/image (sensor_msgs/Image) → face_detection_node
  ↓ face_detections (std_msgs/String)
face_recognition_node
  ↓ face_identities (std_msgs/String)
gaze_analysis_node
  ↓ gaze_status (std_msgs/String)
engagement_manager_node
  ↓ face_event, gaze_event (std_msgs/String)
```

## 依存関係

```bash
# Python依存関係
pip install face_recognition opencv-python numpy

# ROS2依存関係
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

## ビルド方法

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_face_engagement_detector
source install/setup.bash
```

## 使用方法

### 1. 全ノード同時起動（推奨）

```bash
# 通常の複数プロセス起動
ros2 launch susumu_face_engagement_detector simple_launch.py

# 1プロセス内で効率的起動（ComposableNode使用）
ros2 launch susumu_face_engagement_detector multi_node_launch.py
```

### 2. 個別ノード起動

```bash
# 顔検出ノード
ros2 run susumu_face_engagement_detector face_detection_node

# 顔認識ノード
ros2 run susumu_face_engagement_detector face_recognition_node

# 注視判定ノード
ros2 run susumu_face_engagement_detector gaze_analysis_node

# 管理ノード
ros2 run susumu_face_engagement_detector engagement_manager_node
```

### 3. 元の統合ノード起動

```bash
ros2 run susumu_face_engagement_detector face_engagement_node
```

## パラメータ設定

### face_detection_node
- `image_topic`: 入力画像トピック名 (デフォルト: `/image`)
- `detection_model`: 顔検出モデル `hog` or `cnn` (デフォルト: `hog`)

### face_recognition_node
- `known_faces_dir`: 既知の顔画像ディレクトリ (デフォルト: `known_faces`)
- `match_tolerance`: 顔認識の閾値 (デフォルト: `0.6`)

### gaze_analysis_node
- `gaze_threshold_px`: 注視判定の閾値（ピクセル） (デフォルト: `50`)
- `gaze_duration`: 注視判定の持続時間（秒） (デフォルト: `2.0`)

### engagement_manager_node
- `face_timeout`: 顔喪失のタイムアウト（秒） (デフォルト: `1.0`)

## 出力メッセージ

### face_event
- `{face_id}:DETECTED` - 顔検出
- `{face_id}:LOST` - 顔喪失

### gaze_event
- `{face_id}:ENGAGED` - 注視開始
- `{face_id}:DISENGAGED` - 注視終了

## 既知の顔画像設定

```bash
mkdir known_faces
# 既知の人物の顔画像を known_faces/ ディレクトリに配置
# ファイル名がface_idとして使用されます
```

## トピック監視例

```bash
# 顔イベント監視
ros2 topic echo /face_event

# 注視イベント監視
ros2 topic echo /gaze_event

# 内部データ監視
ros2 topic echo /face_detections
ros2 topic echo /face_identities
ros2 topic echo /gaze_status
```

## 動作仕様

1. **顔検出**: カメラ画像から顔を検出し、エンコーディングを生成
2. **顔認識**: 
   - 既知の顔画像と一致すれば優先的に識別
   - 過去に検出した顔と一致すれば追跡
   - 新規の場合は新しいIDを割り当て
3. **注視判定**: 顔がカメラ中央を向いて2秒経過で注視状態と判定
4. **イベント発行**: 顔検出/喪失、注視開始/終了時にイベントを発行

## 効率化のポイント

- **モジュール分離**: 各処理を独立したノードに分割
- **ComposableNode**: 1プロセス内で複数ノードを実行可能
- **パイプライン処理**: 顔検出→認識→注視判定の流れを効率化
- **パラメータ調整**: 用途に応じて閾値や設定を調整可能