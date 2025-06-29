# Claude Development Notes

このファイルには、Claude Codeによる開発作業の記録と重要な情報を記載します。

## プロジェクト概要

**プロジェクト名:** Susumu Face Engagement Detector  
**開発期間:** 2025年6月29日  
**開発者:** Claude Code (Anthropic)  
**リポジトリ:** https://github.com/sato-susumu/susumu_face_engagement_detector

## 開発内容

### アーキテクチャ設計
元の統合型ノード（face_engagement_node.py）を以下の4つのノードに分割：

1. **face_detection_node.py** - 顔検出と顔特徴量生成
2. **face_recognition_node.py** - 顔識別（既知顔優先）
3. **gaze_analysis_node.py** - 注視状態判定
4. **engagement_manager_node.py** - 全体状態管理とイベント発行

### 効率化のポイント
- モジュール分離により保守性向上
- ComposableNodeによる1プロセス内実行
- パイプライン処理による効率化
- パラメータファイルによる設定管理

## テスト環境

### テストフレームワーク
- **pytest** を使用（unittestから変更）
- 単体テスト、統合テスト、パフォーマンステストを実装
- モック機能を活用した独立テスト

### テストカバレッジ
- 各ノードの初期化テスト
- メッセージ処理テスト
- エラーハンドリングテスト
- ノード間通信テスト
- 統合パイプラインテスト

## 起動方法

### 全ノード同時起動
```bash
# 通常の複数プロセス起動
ros2 launch susumu_face_engagement_detector simple_launch.py

# 1プロセス内で効率的起動
ros2 launch susumu_face_engagement_detector multi_node_launch.py
```

### 個別ノード起動
```bash
ros2 launch susumu_face_engagement_detector face_detection_only.launch.py
ros2 launch susumu_face_engagement_detector face_recognition_only.launch.py
ros2 launch susumu_face_engagement_detector gaze_analysis_only.launch.py
ros2 launch susumu_face_engagement_detector engagement_manager_only.launch.py
```

### 元の統合ノード起動
```bash
ros2 run susumu_face_engagement_detector face_engagement_node
```

## テスト実行

```bash
# 全テスト実行
pytest

# 単体テストのみ
pytest -m "not integration"

# 統合テストのみ
pytest -m integration

# 詳細出力
pytest -v

# 特定のテストファイル
pytest test/test_face_detection_node.py
```

## トピック構成

```
/image → face_detection_node → face_detections
              ↓
face_recognition_node → face_identities
              ↓
gaze_analysis_node → gaze_status
              ↓
engagement_manager_node → face_event, gaze_event
```

## 設定ファイル

- `config/face_detection_params.yaml` - 顔検出設定
- `config/face_recognition_params.yaml` - 顔認識設定
- `config/gaze_analysis_params.yaml` - 注視判定設定
- `config/engagement_manager_params.yaml` - 管理ノード設定

## 重要な実装詳細

### メッセージフォーマット
- **face_detections**: `face_idx|center_x|center_y|width|height|frame_width|frame_height|feature_csv`
- **face_identities**: `face_id|center_x|center_y|width|height|frame_width|frame_height`
- **gaze_status**: `face_id|status|center_x|center_y|width|height`
- **face_event**: `face_id:DETECTED|LOST`
- **gaze_event**: `face_id:ENGAGED|DISENGAGED`

### パラメータ
- `image_topic`: 入力画像トピック名 (デフォルト: `/image`)
- `detection_model`: 顔検出モデル `hog`/`cnn` (デフォルト: `hog`)
- `known_faces_dir`: 既知の顔画像ディレクトリ (デフォルト: `known_faces`)
- `match_tolerance`: 顔認識の閾値 (デフォルト: `0.6`)
- `gaze_threshold_px`: 注視判定の閾値 (デフォルト: `50`)
- `gaze_duration`: 注視判定の持続時間 (デフォルト: `2.0`)
- `face_timeout`: 顔喪失のタイムアウト (デフォルト: `1.0`)

## 今後の改善点

1. **カスタムメッセージ型の導入**
   - String型の代わりに構造化されたメッセージ型を定義
   - より型安全な通信の実現

2. **パフォーマンス最適化**
   - 顔検出の頻度調整
   - エンコーディングキャッシュ機能
   - 並列処理の最適化

3. **機能拡張**
   - 複数カメラ対応
   - 顔の向き検出精度向上
   - 感情認識機能の追加

## トラブルシューティング

### よくある問題
1. **face_recognition依存関係エラー**
   ```bash
   pip install face_recognition opencv-python numpy
   ```

2. **ROS2パッケージ依存関係エラー**
   ```bash
   sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
   ```

3. **known_facesディレクトリが見つからない**
   ```bash
   mkdir known_faces
   # 既知の人物の顔画像を配置
   ```

## Git管理

- **初回コミット**: 2025年6月29日
- **ブランチ**: main
- **リモートリポジトリ**: https://github.com/sato-susumu/susumu_face_engagement_detector.git

### コミット履歴
1. Initial commit: Multi-node face engagement detection system
2. Update README.md: Add development status notice

---

**開発完了日**: 2025年6月29日  
**Claude Code バージョン**: Sonnet 4 (claude-sonnet-4-20250514)