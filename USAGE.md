# 使用方法ガイド

## 起動方法

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

## モニタリングツール

システムの動作状況をリアルタイムで監視できるCLIモニタリングツールが含まれています。

```bash
# 基本モニタリング
ros2 run susumu_face_engagement_detector monitoring_node

# ランチファイル使用（推奨）
ros2 launch susumu_face_engagement_detector monitoring.launch.py

# カスタム設定
ros2 launch susumu_face_engagement_detector monitoring.launch.py refresh_rate:=0.5 show_content:=false log_to_file:=true

# 異なる画像トピックを使用
ros2 launch susumu_face_engagement_detector monitoring.launch.py image_topic:=/image
```

### モニタリング機能
- **ノード状態監視**: 各ノードのアクティブ/非アクティブ状態
- **トピック統計**: メッセージ数、Hz、サイズ、最終受信時刻
- **パフォーマンス統計**: 入力レート、出力レート、処理効率
- **データフロー表示**: パイプライン全体の可視化
- **メッセージ内容プレビュー**: 最新メッセージの内容確認

### モニタリングパラメータ
- `refresh_rate`: 画面更新間隔（秒、デフォルト: 1.0）
- `show_content`: メッセージ内容表示（デフォルト: true）
- `log_to_file`: ログファイル出力（デフォルト: false）
- `image_topic`: 入力画像トピック名（デフォルト: /camera/color/image_raw）

## パラメータ設定

### face_detection_node
- `image_topic`: 入力画像トピック名 (デフォルト: `/camera/color/image_raw`)
- `detection_model`: 顔検出モデル `hog` or `cnn` (デフォルト: `hog`)

### face_recognition_node
- `known_faces_dir`: 既知の顔画像ディレクトリ (デフォルト: `known_faces`)
- `match_tolerance`: 顔認識の閾値 (デフォルト: `0.6`)

### gaze_analysis_node
- `gaze_threshold_px`: 注視判定の閾値（ピクセル） (デフォルト: `50`)
- `gaze_duration`: 注視判定の持続時間（秒） (デフォルト: `2.0`)

### engagement_manager_node
- `face_timeout`: 顔喪失のタイムアウト（秒） (デフォルト: `1.0`)

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

## カスタム設定例

### 高速処理設定
```bash
ros2 launch susumu_face_engagement_detector simple_launch.py \
  detection_model:=hog \
  gaze_threshold_px:=30 \
  face_timeout:=0.5
```

### 高精度設定
```bash
ros2 launch susumu_face_engagement_detector simple_launch.py \
  detection_model:=cnn \
  match_tolerance:=0.5 \
  gaze_duration:=3.0
```