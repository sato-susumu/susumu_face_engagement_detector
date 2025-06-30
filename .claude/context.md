# プロジェクトコンテキスト

## プロジェクト概要

**プロジェクト名:** Susumu Face Engagement Detector  
**目的:** ROS2環境における顔認識・注視状態検出システム  
**開発環境:** Ubuntu 22.04, ROS2 Humble, Python 3.10+

## 技術スタック

- **ROS2 Humble** - ロボット用ミドルウェア
- **OpenCV** - 画像処理
- **face_recognition** - 顔認識ライブラリ
- **dlib** - 機械学習ライブラリ
- **pytest** - テストフレームワーク

## アーキテクチャ制約

### ノード構成
1. **face_detection_node** - 顔検出と特徴量生成
2. **face_recognition_node** - 顔識別処理
3. **gaze_analysis_node** - 注視状態判定
4. **engagement_manager_node** - 状態管理とイベント発行

### 通信制約
- **String型メッセージ** - カスタムメッセージ型は未使用
- **CSV形式データ** - 顔特徴量は128次元のCSV形式
- **パイプライン処理** - 順次データ流れ

### パフォーマンス制約
- **リアルタイム処理** - 30fps対応
- **メモリ効率** - 128次元特徴量による軽量化
- **CPU最適化** - HOGモデルをデフォルト使用

## 依存関係

### システム依存
```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

### Python依存
```bash
pip install face_recognition opencv-python numpy
```

### ディレクトリ構造
- `known_faces/` - 既知の人物の顔画像
- `config/` - 各ノードの設定ファイル
- `launch/` - ROS2ランチファイル
- `test/` - テストコード

## 運用制約

### セキュリティ
- 顔画像データの適切な管理
- プライバシー保護の考慮
- ローカル処理による情報漏洩防止

### 互換性
- ROS2 Humble専用
- Ubuntu 22.04推奨
- Python 3.10以上必須

### 既知の制限
- 複数カメラ同時使用不可
- 顔の向き検出精度に制限
- 照明条件による認識精度変動