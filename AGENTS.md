# Claude Development Notes

このパッケージは Phase 0 (2025-06) で発足し、Phase 5+ (2026-06) で全面刷新を経た現状を反映した開発ノートです。古い記録は git 履歴を参照してください。

## アーキテクチャ (現状)

カメラ入力から ROS4HRI 準拠の engagement 信号までを以下のノードで実現:

| ノード | 役割 | 主トピック | バックエンド |
|---|---|---|---|
| `face_detection_node` | 顔検出 | `/face_detections_vision` (Detection2DArray) | dlib_hog / dlib_cnn / yunet |
| `face_recognition_node` | 顔認識・ID追跡 | `/humans/faces/tracked` (IdsList) | dlib_128d |
| `head_pose_node` | 頭部姿勢 | `/humans/faces/head_pose` (PoseStamped) | mediapipe_pnp |
| `gaze_node` | 視線推定 | `/humans/faces/gaze` (Vector3Stamped) | openvino_adas (optional) |
| `expression_node` | 表情 | `/humans/faces/expression` (Expression) | hsemotion (Apache 2.0) |
| `engagement_node` | エンゲージメント | `/humans/persons/<id>/engagement_status` (EngagementLevel) | Concentration Index + EMA + ヒステリシス |

バックエンドはROS パラメータで切替。プラグイン実装は `susumu_face_engagement_detector/backends/`。

## 主な作業フロー

```bash
# ビルド
colcon build --packages-select susumu_face_engagement_detector

# 起動
ros2 launch susumu_face_engagement_detector engagement_pipeline.launch.py

# テスト
make test

# 評価・可視化を一発で
make outputs   # → outputs/baselines/, outputs/figures/, outputs/reports/REPORT.md
```

評価ハーネスは `eval/`、成果物は `outputs/` に集約。詳細は:

- [`README.md`](README.md) — 概要・クイックスタート
- [`docs/REVAMP_PLAN.md`](docs/REVAMP_PLAN.md) — Phase 0-5 の意思決定経緯
- [`eval/README.md`](eval/README.md) — 評価ハーネス
- [`outputs/README.md`](outputs/README.md) — 成果物の保存先構成

## 重要な実装メモ

### メッセージ標準
- **`vision_msgs/Detection2DArray`** と **`hri_msgs`** (REP-155) に準拠
- `BoundingBox2D.center` は `Pose2D` で、座標は `bb.center.position.x/y` (内側にネスト)
- `hri_msgs/EngagementLevel` は 5値 (UNKNOWN/DISENGAGED/ENGAGING/ENGAGED/DISENGAGING)

### テスト戦略
- `test/unit/` — backend / metrics / scorer の純Pythonテスト
- `test/integration/` — ROS 2 ノード起動を subprocess に分離してテスト
  - 理由: 過去にlegacyテストが `sys.modules['cv2'] = MagicMock()` で汚染していたため、サブプロセス起動で状態を完全隔離する
- pytest コマンド: `testpaths = test/unit test/integration` を `pytest.ini` で明示

### ライセンス境界
- **コード**は Apache 2.0
- **重み**は backend ごとに異なる:
  - YuNet / HSEmotion / MediaPipe / dlib → 商用可
  - InsightFace (SCRFD/ArcFace) / Ultralytics YOLO → 商用不可 / 別契約必要
- 詳細は README の「ユースケース別の構成」セクション

### engagement score
Altuwairqi らの Concentration Index 式:
```
raw   = EmotionWeight / max(EmotionWeights)
filt  = EMA(raw, α=0.3)
gated = filt if HeadPoseGate else filt × 0.3
```
全パラメータは `engagement_node` の ROS param で調整可能。
