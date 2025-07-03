# Susumu Face Engagement Detector

⚠️ **開発中のプロジェクトです** ⚠️

このプロジェクトは現在開発中です。機能の追加や変更が頻繁に行われる可能性があります。

---

顔検出・認識・注視判定を行うROS2パッケージです。効率的な動作のため、複数のノードに分割して実装されています。

## 🎯 概要

このシステムは以下の機能を提供します：

- **👤 顔検出**: カメラ画像からリアルタイムで顔を検出
- **🔍 顔認識**: 既知の人物の識別と新規人物の追跡
- **👁️ 注視分析**: カメラに向けた注視状態の判定
- **📢 イベント通知**: 顔の出現・消失、注視開始・終了の通知
- **📊 リアルタイム監視**: システム全体の動作状況監視

## 🏗️ アーキテクチャ

### ノード構成
- **face_detection_node**: 画像から顔を検出し、顔特徴量を生成
- **face_recognition_node**: 顔特徴量から人物を識別
- **gaze_analysis_node**: 顔の位置から注視状態を判定
- **engagement_manager_node**: 全体の状態管理とイベント発行

### データフロー
```
📹 Camera → 👤 Detection → 🔍 Recognition → 👁️ Gaze → 📢 Events
```

## 🚀 クイックスタート

### 1. インストール

詳細は [INSTALLATION.md](INSTALLATION.md) を参照してください。

```bash
# 依存関係インストール
pip install face_recognition opencv-python numpy
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs

# ビルド
cd ~/ros2_ws
colcon build --packages-select susumu_face_engagement_detector
source install/setup.bash
```

### 2. 起動

```bash
# 全ノード起動
ros2 launch susumu_face_engagement_detector simple_launch.py

# モニタリング
ros2 run susumu_face_engagement_detector monitoring_node
```

### 3. 動作確認・テスト

システムの動作確認には疑似カメラが使用できます：

```bash
# 疑似カメラでのテスト
python susumu_face_engagement_detector/test_camera_node.py

# 包括的検証
python scripts/test_pipeline.py
```

詳細は [TESTING.md](TESTING.md) を参照してください。

## 📚 ドキュメント

| ドキュメント | 内容 |
|---|---|
| [INSTALLATION.md](INSTALLATION.md) | 依存関係とビルド方法 |
| [USAGE.md](USAGE.md) | 起動方法とパラメータ設定 |
| [API.md](API.md) | トピック仕様とメッセージフォーマット |
| [TESTING.md](TESTING.md) | 動作検証とテスト方法 |

## 💡 特徴

### 🔧 モジュラー設計
- 各機能を独立したノードに分割
- ComposableNodeによる効率的な実行
- パイプライン処理による高スループット

### 📊 リアルタイム監視
- CLIベースの包括的監視ツール
- ノード状態、トピック統計、パフォーマンス指標
- 視覚的なデータフロー表示

### 🎯 RViz可視化
- 顔検出結果のリアルタイム3D表示
- 複数人の顔を色分けして同時表示
- 顔認識結果の視覚的確認（既知・未知・ユーザー別）
- アノテーション付き画像の表示

### 🧪 包括的テスト
- 疑似カメラによる自動検証
- 著作権フリーの合成テスト画像
- 7パターンの検証シナリオ

### ⚙️ 柔軟な設定
- パラメータによるカスタマイズ
- 複数の起動方法
- 用途に応じた最適化設定

## 📊 出力例

### イベント出力
```bash
# 顔イベント
ros2 topic echo /face_event
# → user_1:DETECTED
# → user_1:LOST

# 注視イベント  
ros2 topic echo /gaze_event
# → user_1:ENGAGED
# → user_1:DISENGAGED
```

### モニタリング画面
```
📊 Node Status:
  face_detection    🟢 ACTIVE
  face_recognition  🟢 ACTIVE
  gaze_analysis     🟢 ACTIVE
  engagement_manager🟢 ACTIVE

📡 Topic Status:
  /face_detections     ✅ 15 msgs, 3.2 Hz
  /face_identities     ✅ 12 msgs, 2.8 Hz
  /gaze_status         ✅ 15 msgs, 3.2 Hz
  /face_event          ✅ 3 msgs, 0.5 Hz
```

## 🎮 使用例

### 基本使用
```bash
# システム起動
ros2 launch susumu_face_engagement_detector simple_launch.py

# 別ターミナルでモニタリング
ros2 run susumu_face_engagement_detector monitoring_node
```

### カスタム設定
```bash
# 高速処理設定
ros2 launch susumu_face_engagement_detector simple_launch.py \
  detection_model:=hog gaze_threshold_px:=30

# 高精度設定
ros2 launch susumu_face_engagement_detector simple_launch.py \
  detection_model:=cnn match_tolerance:=0.5
```

### RViz可視化
```bash
# 顔検出システム起動（RViz可視化対応）
ros2 launch susumu_face_engagement_detector face_detection_with_rviz.launch.py

# 別ターミナルでRViz起動（GUI環境の場合）
rviz2 -d config/face_detection_rviz.rviz

# 可視化トピック確認
ros2 topic list | grep -E "(face_detection|identity|markers)"
```

### テスト・検証
```bash
# 動的顔生成テスト
python susumu_face_engagement_detector/test_camera_node.py --ros-args -p fps:=5.0

# 複数顔画像テスト
python susumu_face_engagement_detector/test_camera_node.py --ros-args \
  -p test_mode:=file -p image_file:=test_images/multiple_faces_test.jpg

# パイプライン検証
python scripts/test_pipeline.py
```

## 📄 ライセンス

Apache License 2.0

---

詳細な使用方法は各ドキュメントを参照してください。