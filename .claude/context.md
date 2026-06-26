# プロジェクトコンテキスト

## 概要

**プロジェクト名:** Susumu Face Engagement Detector
**目的:** カメラ映像から顔検出 → 認識 → 頭部姿勢 → 視線 → 表情 → エンゲージメント検出を、ROS4HRI (REP-155) 準拠のメッセージで提供する ROS 2 パッケージ
**開発環境:** Ubuntu 22.04, ROS 2 Humble, Python 3.10+
**最終更新:** 2026-06-26 (Phase 5+ 完了)

## 採用技術 (Phase 5+)

| 機能 | 採用バックエンド | ライセンス |
|---|---|---|
| 顔検出 | YuNet (OpenCV Zoo) / dlib HOG fallback | Apache 2.0 / Boost SL |
| 顔認識 | dlib 128-D | Boost SL + パブリックドメイン重み |
| 頭部姿勢 | MediaPipe Face Mesh + solvePnP | Apache 2.0 |
| 視線方向 | MediaPipe Iris (FaceMesh refine_landmarks) | Apache 2.0 |
| 表情認識 | HSEmotion ONNX (EfficientNet/AffectNet 8-class) | Apache 2.0 |
| エンゲージメント | Altuwairqi Concentration Index + EMA + 5値ヒステリシス | (実装) Apache 2.0 |

すべて Apache 2.0 / MIT / Boost SL のみで構成 — 商用展開可能。

## アーキテクチャ

6 個の独立ノードがパイプラインを構成:

```
camera/color/image_raw
  → face_detection_node      → /face_detections_vision (vision_msgs/Detection2DArray)
  → face_recognition_node    → /humans/faces/tracked    (hri_msgs/IdsList)
  → head_pose_node           → /humans/faces/head_pose  (geometry_msgs/PoseStamped)
  → gaze_node                → /humans/faces/gaze       (geometry_msgs/Vector3Stamped)
  → expression_node          → /humans/faces/expression (hri_msgs/Expression)
  → engagement_node          → /humans/persons/<id>/engagement_status (hri_msgs/EngagementLevel 5値)
```

バックエンド実装は `susumu_face_engagement_detector/backends/` 配下 (detection / recognition / headpose / gaze / expression)。各ノードは ROS param で backend を切替可能。

## 主要パフォーマンス指標 (WIDER FACE val 3,226 画像)

| Backend  | AP@0.5 | mean / p95 latency |
|---|---:|---:|
| dlib HOG (旧ベースライン) | 13.70% | 232.6 / 406.8 ms |
| **YuNet (現行)** | **58.79%** | **23.4 / 63.9 ms** |

LFW 1:1 verification: **98.50%** accuracy (AUC 0.9974, 4.1 ms/embedding)。

全数値・図 → [`outputs/reports/REPORT.md`](../outputs/reports/REPORT.md)

## 依存関係

### システム

```bash
sudo apt install \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-hri-msgs \
    ros-humble-rosbag2-storage-mcap
```

### Python

```bash
pip3 install --user mediapipe hsemotion-onnx face_recognition opencv-contrib-python
```

### モデル重み

- `~/models/face_detection/face_detection_yunet_2023mar.onnx` (YuNet, ~230 KB)
- `~/.hsemotion/enet_b0_8_best_afew.onnx` (HSEmotion, ~16 MB)

### 評価データセット

- `~/datasets/wider_face/` (WIDER FACE val, 検出評価用)
- LFW pairs は `sklearn.datasets.fetch_lfw_pairs` で自動取得

## ディレクトリ構造

```
.
├── susumu_face_engagement_detector/  # ノード実装 (6) + backends (5)
├── eval/                             # 評価ハーネス
├── outputs/                          # JSON baselines / PNG figures / REPORT.md
├── launch/                           # 4 launch ファイル
├── config/                           # 3 YAML パラメータ
├── test/                             # unit + integration
├── docs/REVAMP_PLAN.md               # 変革計画
├── .github/workflows/ci.yml          # CI
└── README.md / CLAUDE.md / Makefile
```

## 開発フェーズ履歴

- **Phase 0**: 設計・3 並列技術調査 → [`docs/REVAMP_PLAN.md`](../docs/REVAMP_PLAN.md)
- **Phase 1**: 評価ハーネス、ベースライン取得、`vision_msgs` 並行発行
- **Phase 2**: YuNet 化 (AP 4.3 倍)、`hri_msgs/IdsList` 並行発行
- **Phase 3**: head_pose / gaze / expression ノード新設
- **Phase 4**: engagement_node 再設計 (CI + 5値 EngagementLevel)
- **Phase 5**: README/Makefile 整備
- **Phase 5+**: `outputs/` 集約、可視化 (matplotlib)、レガシー完全削除

## 関連ドキュメント

- [`README.md`](../README.md) — クイックスタート・公開仕様
- [`CLAUDE.md`](../CLAUDE.md) — 開発ノート
- [`docs/REVAMP_PLAN.md`](../docs/REVAMP_PLAN.md) — 変革計画と意思決定経緯
- [`eval/README.md`](../eval/README.md) — 評価ハーネス操作
- [`outputs/README.md`](../outputs/README.md) — 成果物保存先構成
