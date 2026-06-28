# Susumu Face Engagement Detector — 根本変革計画 (Revamp Plan)

**作成日**: 2026-06-26
**最終更新**: 2026-06-28 (Phase 5+ 完了、レガシー完全削除済)
**対象バージョン**: ROS 2 Humble (現行) → Humble 維持
**スコープ**: 技術スタック全面刷新 + 検証基盤新設 + メッセージ標準化

## 実施サマリ

| Phase | 状態 | 主な成果 |
|---|---|---|
| 0 | 完了 | 設計、3 並列技術調査、構成C採用、本書 |
| 1 | 完了 | 評価ハーネス、WIDER FACE val 3,226画像で v0 ベースライン取得 (dlib HOG AP@0.5=13.70%)、`vision_msgs` 並行発行 |
| 2 | 完了 | YuNet 統合 (AP **58.79%** = 4.3 倍, latency 10 倍高速)、LFW recognition baseline (**98.50%**)、`hri_msgs/IdsList` 並行発行 |
| 3 | 完了 | `head_pose_node` (MediaPipe PnP) / `expression_node` (HSEmotion ONNX) 新設 |
| 4 | 完了 | `engagement_node` 新設 (Altuwairqi Concentration Index + 5 値 `hri_msgs/EngagementLevel` + EMA + ヒステリシス) |
| 5 | 完了 | README/Makefile 整備、launch 整理 |
| 5+ | 完了 | 成果物の `outputs/` 集約、`eval/visualize.py` で 3 カテゴリの図 PNG 自動生成、レポートに図埋め込み、**レガシー完全削除** (旧ノード/launch/scripts/docs/test_images/test/legacy/build成果物) |

**最終テスト数**: unit / integration とも緑 (最新件数は pytest 出力を参照)。
**成果物**: `outputs/{baselines,figures,reports}/` に集約、`make outputs` 一発で再生成。
**現役ノード**: 6 個 (face_detection / face_recognition / head_pose / gaze / expression / engagement)。`gaze_node` は OpenVINO モデル配置時だけ有効化する optional ノード。
**リポジトリサイズ**: ~3.9 MB (build/install/log 除く)。


---

## 1. 背景と目的

### 1.1 現状の課題

| 領域 | 現状 | 問題 |
|---|---|---|
| 顔検出 | `face_recognition` (dlib HOG/CNN, 2018年頃の世代) | WIDER FACE Hard 圏でほぼ機能しない。ライブラリは2018年から実質メンテ停止 |
| 顔認識 | dlib ResNet-34 128次元埋め込み | LFW 99.38% で飽和、IJB-C等の困難条件で SOTA に大差 |
| 関心度判定 | 顔の中心座標が画面中央近傍か | 顔位置トラッキング+正面顔判定にすぎず、本来のengagementを測れていない |
| メッセージ | `std_msgs/String` にパイプ区切り独自フォーマット | 型安全性ゼロ、`ros2 bag` 再利用困難、可視化ツール非対応、業界標準不在 |
| 検証 | `test_images/` の合成画像を目視 | 評価指標が存在しない。回帰検出不能。「先進化」の優劣を数値で語れない |
| engagement定義 | DETECTED/LOST + ENGAGED/DISENGAGED の2値 | 表情・頭部姿勢の統合という本来の定義から外れる |

### 1.2 変革の目的

1. **「本物のengagement検出」を実装する** — 頭部姿勢・表情を統合した、根拠ある engagement score を出す
2. **検証可能にする** — 公開ベンチマークに対する数値で性能を語れるようにする
3. **業界標準に乗る** — ROS4HRI (REP-155) + `vision_msgs` 採用で、ros4hriエコシステムと相互運用可能にする
4. **商用展開可能なライセンスを維持する** — Apache 2.0 / MIT / Boost のみで構築

### 1.3 採用した方針 (確定済)

- **技術構成**: 構成C「ハイブリッド」(商用安全 + 現代化バランス)
- **対象ハード**: CPU/GPU 両対応、GPU 無時は MediaPipe にfallback
- **ライセンス方針**: 商用利用可能性を残す (Apache 2.0/MIT/Boost のみ)
- **進め方**: Phase 0 → Phase 1 (検証基盤) → Phase 2以降。先に詳細計画 (本書) を確定

---

## 2. 技術選定 (構成C)

### 2.1 採用スタック

| 機能 | 採用技術 | ライセンス | CPU性能 | GPU性能 | 備考 |
|---|---|---|---|---|---|
| 顔検出 (GPU) | YuNet (OpenCV Zoo) | Apache 2.0 | 30+ FPS | 100+ FPS | `hri_face_detect` 採用済。商用OK |
| 顔検出 (高精度) | SCRFD-500M | Apache 2.0 (要重み確認※) | 28ms/VGA | 3.6ms | InsightFace SCRFD はコードApache 2.0だが配布重みは非商用。**自前学習または YuNet で代替**必須 |
| 顔認識 | MagFace (R50) | Apache 2.0 + 重みもApache 2.0 | 25ms | 5ms | LFW 99.83% / CFP-FP 98.46%、ArcFace相当 |
| 顔認識 (fallback) | dlib face_recognition (128次元) | Boost SL + パブリックドメイン | 既存ベースライン維持 | - | 重み完全クリーン |
| 頭部姿勢 | 6DRepNet | MIT | 動作可 | 高速 | AFLW2000 MAE 3.97°、ROS 2ラッパ既存 (`6DRepNet_ros`) |
| 頭部姿勢 (CPU fallback) | MediaPipe Face Landmarker + solvePnP | Apache 2.0 | 30+ FPS | - | MAE 5-10° 程度 |
| 視線推定 | OpenVINO gaze-estimation-adas-0002 | Apache 2.0 | CPU可 | - | 左右目 crop + head pose を入力。簡易ランドマーク推定の代替として optional 採用 |
| 表情認識 | HSEmotion-ONNX | Apache 2.0 (重み含む) | リアルタイム | 高速 | AffectNet 8-class 62.5%, EfficientNet系 |
| トラッキング | ByteTrack (Python実装) | MIT | - | - | ID一貫性、`hri_face_detect` 内のIoUトラッカで代替も可 |

※ SCRFD重みのライセンス再確認は Phase 2 着手時にtask化

### 2.2 メッセージ仕様

| 用途 | 採用msg | 出所 | 備考 |
|---|---|---|---|
| 顔検出結果 | `vision_msgs/Detection2DArray` | ros-perception | 業界標準、可視化ツール対応 |
| バウンディングボックス | `vision_msgs/BoundingBox2D` | ros-perception | - |
| 顔ID/トラッキング | `Detection2D.id` (string) | ros-perception | フレーム間トラッキングID |
| 顔ランドマーク (任意) | `hri_msgs/FacialLandmarks` | ROS4HRI | 478点 |
| 表情 | `hri_msgs/Expression` (categorical) | ROS4HRI | neutral/happy/sad/anger/etc |
| 表情 (連続値) | `hri_msgs/SoftBiometrics` または独自 | ROS4HRI | valence/arousal |
| 頭部姿勢 | TF フレーム `face_<id>` + `geometry_msgs/PoseStamped` | tf2 + standard | ROS4HRI 慣行 |
| エンゲージメント | `hri_msgs/EngagementLevel` (5値) | ROS4HRI | UNKNOWN/DISENGAGED/ENGAGING/ENGAGED/DISENGAGING |
| 既知人物ID | `hri_msgs/IdsList` | ROS4HRI | `/humans/persons/known` 等 |

### 2.3 トピック構造 (ROS4HRI準拠)

```
/camera/color/image_raw           # 入力
/humans/faces/tracked             # IdsList — 現在検出中の顔ID
/humans/faces/<id>/roi            # RegionOfInterestStamped
/humans/faces/<id>/landmarks      # FacialLandmarks (任意)
/humans/faces/<id>/gaze           # Vector3Stamped (任意)
/humans/faces/<id>/expression     # Expression
TF: camera_color_optical_frame → face_<id>  # 頭部6DoF姿勢
/humans/persons/tracked           # IdsList
/humans/persons/<id>/face_id      # std_msgs/String — face_id との関連
/humans/persons/<id>/engagement_status  # EngagementLevel
/humans/persons/<id>/anonymous    # Bool — 既知/未知
```

### 2.4 ノード分割案 (CPU/GPU 自動切替前提)

```
face_detection_node
  ├─ backend: yunet | scrfd_500m | mediapipe
  ├─ param: backend (auto = GPU有→scrfd, 無→yunet)
  ├─ pub: /humans/faces/tracked, /humans/faces/<id>/roi
  └─ pub: vision_msgs/Detection2DArray (互換性のため)

face_recognition_node
  ├─ backend: magface | dlib (fallback)
  ├─ sub: /humans/faces/<id>/roi + cropped image
  └─ pub: /humans/persons/* (person manager 経由)

facial_landmark_node (新設・任意)
  ├─ MediaPipe Face Landmarker
  └─ pub: /humans/faces/<id>/landmarks

head_pose_node (新設)
  ├─ backend: 6drepnet | mediapipe_pnp
  └─ pub: TF face_<id>, PoseStamped

gaze_node (新設・任意)
  ├─ backend: openvino_adas
  ├─ 入力: 左右目 crop + head_pose
  └─ pub: /humans/faces/<id>/gaze

expression_node (新設)
  ├─ HSEmotion-ONNX
  └─ pub: /humans/faces/<id>/expression

person_manager_node
  ├─ 顔ID ↔ person_ID マッピング、known faces照合
  └─ pub: /humans/persons/*

engagement_node (現engagement_manager_node を再定義)
  ├─ 入力: head_pose, expression
  ├─ Concentration Index 計算
  ├─ EMA + ヒステリシス + persistence判定
  └─ pub: /humans/persons/<id>/engagement_status
```

### 2.5 engagement score の定義 (Phase 4で実装)

**Altuwairqi Concentration Index** をベースに、当プロジェクト用に調整:

```
EmotionWeight:
  neutral: 0.9, happy: 0.7, surprise: 0.6, sad: 0.3,
  anger: 0.2, fear: 0.2, disgust: 0.1
HeadPoseGate: yaw ∈ [-15°, +15°] AND pitch ∈ [-10°, +10°] → pass

raw_score = EmotionWeight / max_emotion_weight  # ∈ [0, 1]
filtered_score = EMA(raw_score, α=0.3)
gated_score = filtered_score if HeadPoseGate else filtered_score * 0.3

EngagementLevel:
  gated_score >= 0.66 AND duration >= 2s → ENGAGED
  gated_score >= 0.33                    → ENGAGING
  gated_score < 0.33  AND duration >= 1s → DISENGAGING
  gated_score < 0.10  AND duration >= 2s → DISENGAGED
  顔ロスト                                → UNKNOWN
```

ヒステリシスはEngagementLevel昇格/降格の閾値を非対称化 (昇格0.66/降格0.50など) で実装。

---

## 3. 検証基盤の設計 (Phase 1 の本体)

### 3.1 ディレクトリ構造

```
susumu_face_engagement_detector/
├── eval/                            # 新設
│   ├── README.md                    # 評価実行手順
│   ├── datasets/
│   │   ├── wider_face/              # WIDER FACE downloader & GT parser
│   │   ├── lfw/                     # LFW downloader & pair list
│   │   ├── aflw2000/                # 頭部姿勢
│   │   ├── affectnet/               # 表情 (要申請)
│   │   └── daisee/                  # engagement
│   ├── rosbag_converters/
│   │   ├── images_to_rosbag.py      # image_publisher経由でbag作成
│   │   ├── video_to_rosbag.py
│   │   └── synth_scenarios.py       # 自作シナリオ生成
│   ├── runners/
│   │   ├── run_detection_eval.py    # bag再生 → 出力収集 → mAP計算
│   │   ├── run_recognition_eval.py  # 1:1 verification accuracy
│   │   ├── run_headpose_eval.py     # MAE per axis
│   │   ├── run_emotion_eval.py      # confusion matrix, F1
│   │   ├── run_engagement_eval.py   # DAiSEE accuracy
│   │   └── run_tracking_eval.py     # MOTA/IDF1/HOTA (py-motmetrics)
│   ├── metrics/
│   │   ├── detection.py             # AP, mAP
│   │   ├── recognition.py
│   │   ├── headpose.py
│   │   ├── tracking.py
│   │   └── latency.py               # end-to-end latency
│   ├── baselines/                   # 数値スナップショット
│   │   ├── v0_dlib_baseline.json    # 現状実装の数値 (Phase 1で取得)
│   │   ├── v1_yunet.json
│   │   └── ...
│   └── reports/
│       └── generate_report.py       # Markdown比較レポート自動生成
├── test/
│   ├── unit/                        # 既存pytest群を移動
│   ├── integration/                 # launch_pytest
│   │   ├── test_pipeline_e2e.py
│   │   ├── test_msg_contract.py     # msg型契約テスト
│   │   └── fixtures/                # 小規模bag
│   └── conftest.py
└── scripts/                         # 既存 (整理)
```

### 3.2 評価データセットの整備

| データセット | 用途 | 規模 | ライセンス | 入手 |
|---|---|---|---|---|
| WIDER FACE | 顔検出 | 32,203画像 | 学術用 | 公式DL → 自作スクリプトで rosbag化 |
| LFW | 顔認識 1:1 | 13,233画像/5,749名 | 学術用 | scikit-learn `fetch_lfw_pairs` |
| CFP-FP | 顔認識 (横顔) | 7,000ペア | 学術用 | 公式DL |
| AFLW2000-3D | 頭部姿勢 | 2,000画像 | 学術用 | 公式DL |
| AffectNet | 表情 | 420Kラベル | 学術用 (PI申請) | PI署名後 |
| DAiSEE | engagement | 9,068動画/112名 | 研究用 (form申請) | フォーム |
| **自作シーン** | end-to-end | 10-30本 | 自由 | 当チームで撮影 |

公開rosbag2配布は確認できなかったため、**`image_publisher` + `ros2 bag record` で自前変換**するのが現実解 (ROS 2エコシステムのギャップ)。

### 3.3 評価実行フロー

```
[Dataset] → [rosbag_converter.py] → [scenario.bag]
                                          ↓
                              [ros2 launch eval_pipeline.launch.py]
                                          ↓
                       [bag再生 + 全ノード起動 + 出力収集]
                                          ↓
                          [runners/run_*_eval.py で指標計算]
                                          ↓
                          [baselines/ にJSON保存 + 前回比diff]
                                          ↓
                          [reports/ で Markdown 自動生成]
```

CI (GitHub Actions) では小規模なfixtureだけ走らせ、フルベンチは手動コマンド (`make eval-full`) で。

### 3.4 統合テスト雛形 (`launch_pytest`)

```python
# test/integration/test_pipeline_e2e.py
import launch_pytest
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node

@launch_pytest.fixture
def launch_description():
    return LaunchDescription([
        Node(package='susumu_face_engagement_detector',
             executable='face_detection_node',
             parameters=[{'backend': 'yunet'}]),
        # ... 他のノード
    ])

@pytest.mark.launch(fixture=launch_description)
def test_detection_publishes_on_known_image(rosbag_player_fixture):
    # 1. fixture bag を再生
    # 2. /humans/faces/tracked を購読して非空であることを確認
    # 3. /humans/faces/<id>/roi が GT bbox と IoU > 0.5
    ...

@pytest.mark.launch(fixture=launch_description, shutdown=True)
def test_clean_shutdown():
    # post-shutdown: ノードがエラーなしで終了したこと
    ...
```

`realsense-ros` の `RosbagManager` + `RsTestNode` + `process_data()` パターンを参考実装として移植する。

### 3.5 CI 設定

`.github/workflows/ci.yml`:

```yaml
name: CI
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: ros-tooling/setup-ros@v0.7
        with: { required-ros-distributions: humble }
      - uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: susumu_face_engagement_detector
          target-ros2-distro: humble
          coverage-result: true
      - uses: codecov/codecov-action@v4
```

### 3.6 性能計測

- **PR段階**: `ros2 topic hz/delay` で基礎チェック (test内で assertion)
- **本格時**: `ros2_tracing` (Iron以降標準、再ビルド不要)、深掘りで CARET (Tier IV)
- **負荷ベンチマーク**: `performance_test` (Apex.AI v2.3.0) で pub-to-sub latency
- **可視化**: PlotJuggler (時系列)、Foxglove (MCAP)、`hri_rviz`

---

## 4. Phase 別実行計画

### Phase 0: 棚卸し・基盤合意 (1週間, 着手済)

- [x] 現状コードレビュー
- [x] 技術スタック調査 (3並列リサーチ)
- [x] 構成C採用決定
- [ ] **本書 (REVAMP_PLAN.md) のレビューと確定**
- [ ] ライセンスチェックリスト確認
- [ ] 評価データセット入手手配 (DAiSEE/AffectNetは申請に時間かかる)
- [ ] Phase 1 着手判断

**成果物**: 本書、ライセンス整理表

### Phase 1: 検証基盤の整備 (2-3週間) ← **最優先**

目的: **既存実装のベースライン数値を取得できる状態にする**

- [ ] `package.xml` に `vision_msgs`, `hri_msgs` を追加、依存解決
- [ ] 既存 `String` 出力に**並行して** `vision_msgs/Detection2DArray` も発行 (互換維持)
- [ ] `eval/datasets/` に WIDER FACE / LFW / 自作シーンのダウンローダ実装
- [ ] `eval/rosbag_converters/images_to_rosbag.py` 実装 (image_publisher経由)
- [ ] `eval/runners/run_detection_eval.py` 実装 (bag再生 → 出力収集 → mAP)
- [ ] `eval/metrics/detection.py` (AP/mAP)、`eval/metrics/latency.py`
- [ ] `test/integration/test_pipeline_e2e.py` の雛形 (`launch_pytest`)
- [ ] `realsense-ros` パターンを参考に `RosbagManager` 移植
- [ ] GitHub Actions CI セットアップ (`action-ros-ci`)
- [ ] **`eval/baselines/v0_dlib_baseline.json` を取得・コミット**
- [ ] `eval/reports/generate_report.py` で初回レポート出力

**Definition of Done**:
- `make eval-detection-baseline` で WIDER FACE val Easy/Medium/Hard の AP が出力される
- CI が緑で通る
- `eval/baselines/v0_dlib_baseline.json` がコミット済
- README に「ベースライン v0: WIDER Easy 36.5% / Hard 0%」等の数値が記載

### Phase 2: 検出・認識の現代化 (2-3週間)

- [ ] `hri_face_detect` 採用検討 — そのまま採用するか自作維持するか判断
- [ ] `face_detection_node` リファクタ: backend切替対応 (`yunet` / `scrfd` / `mediapipe`)
- [ ] YuNet統合 (OpenCV Zoo)
- [ ] (任意) SCRFD-500M 統合 + 重みライセンス再確認
- [ ] `face_recognition_node` リファクタ: backend切替 (`magface` / `dlib`)
- [ ] MagFace ONNX 重み入手・統合
- [ ] **Phase 1 の評価ハーネスで差分検証**
  - WIDER FACE val: dlib HOG → YuNet → SCRFD で AP 比較
  - LFW pairs: dlib → MagFace で accuracy 比較
- [ ] `eval/baselines/v1_yunet_magface.json` 取得・コミット
- [ ] 性能リポート Markdown 自動生成

**Definition of Done**:
- WIDER FACE Hard AP が v0 (≈0%) → v1 (>50%) に改善された数値が出る
- LFW accuracy が v0 (99.38%) → v1 (>99.7%) に改善
- 全テスト緑

### Phase 3: 姿勢・表情ノード新設 (3-4週間)

- [ ] `head_pose_node` 新設
  - 6DRepNet ノード化 (`6DRepNet_ros` 参照)
  - MediaPipe + solvePnP fallback
  - TF + PoseStamped 配信
  - AFLW2000-3D で MAE 評価
- [ ] `expression_node` 新設
  - HSEmotion-ONNX 統合
  - `hri_msgs/Expression` 配信
  - AffectNet で confusion matrix
- [ ] (任意) `facial_landmark_node` (MediaPipe Face Landmarker)
- [ ] 各ノード単体で評価ベースライン取得

**Definition of Done**:
- head pose MAE: AFLW2000 で <5°
- expression: AffectNet 8-class で >55%
- 全ノードがCPU/GPU両モードで起動可

### Phase 4: engagement再定義 (2-3週間)

- [ ] 旧 `engagement_manager_node` の2値判定ロジックを deprecate
- [ ] 新 `engagement_node` 実装
  - Concentration Index 実装 (Altuwairqi式)
  - EMA + ヒステリシス
  - `hri_msgs/EngagementLevel` 5値配信
  - persistence判定 (2秒持続)
- [ ] パラメータチューニング用のparam yaml整備
- [ ] DAiSEE での評価
  - DAiSEE engagement 4階級 ↔ EngagementLevel 5値 のマッピング
  - accuracy / MSE 計算
- [ ] `eval/baselines/v3_engagement.json` コミット
- [ ] `hri_engagement` パッケージの指標とも比較

**Definition of Done**:
- DAiSEE engagement 4クラス accuracy がベースライン (現状: 評価不能) → >55%
- `hri_msgs/EngagementLevel` が正しく5値で遷移する単体テストが緑

### Phase 5: ドキュメント・公開準備 (1週間)

- [ ] README全面改訂
  - 「商用ユースケース」「研究ユースケース」別の起動例
  - ベースライン vs 現バージョンの数値比較表
- [ ] `docs/` 配下整理 (PERFORMANCE_OPTIMIZATION.md 更新)
- [ ] 移行ガイド: 旧 String I/F → 新 vision_msgs/hri_msgs
- [ ] 旧 `face_engagement_node.py` / 旧 `engagement_manager_node.py` の最終deprecate判断
- [ ] (任意) ros4hri エコシステムへの貢献PR検討

**Definition of Done**:
- READMEから新規ユーザーが5分以内に動作確認できる
- ベースライン数値が公開され、PR毎に regression 検出可能
- v1.0 タグ打ち

---

## 5. リスクと対策

| リスク | 影響 | 対策 |
|---|---|---|
| SCRFD重みが非商用判定 | Phase 2で代替必要 | YuNet (Apache 2.0) にfallback。事前に学習レシピのみ Apache 2.0 として独自学習も検討 |
| MagFace重みの実体ライセンス | Phase 2でdlibに戻す可能性 | dlib (Boost SL, パブリックドメイン重み) を fallback として常設 |
| データセット入手の遅延 | Phase 1 完了が遅れる | DAiSEE/AffectNetは申請が必要 — Phase 0で先に申請開始 |
| 既存String I/Fの後方互換 | 既存ダウンストリーム破壊 | Phase 1-2 では並行配信、Phase 5 で非推奨化、v2.0 で削除 |
| GPU/CPU切替の複雑化 | コード量増 | 各 backend モジュールを薄く保ち、ノード本体は I/F 共通化 |
| ROS4HRI hri_msgs の Humble対応 | パッケージビルド失敗 | 2026-02-10 リリース版 (2.3.2) を採用、依存解決時に確認 |
| Python rclpy で zero-copy 不可 | ComposableNode化の効果が出ない | ComposableNode化はやらない。最適化が必要な箇所のみ C++ 移植 |

---

## 6. 当初設計との差分 (なぜ変更したか)

| 当初提案 (v1) | v2 で変更 | 理由 |
|---|---|---|
| カスタムmsg定義を新設 | `vision_msgs` + `hri_msgs` 採用 | ROS4HRI標準が既存、独自路線は孤立を招く |
| YOLOv8-face / RetinaFace / SCRFD を並列検討 | YuNet主軸、SCRFDは要再検討 | YOLOv8はAGPL-3.0、SCRFD重みは非商用 — 商用安全な選択肢が限られる |
| InsightFace ArcFace を主推奨 | MagFace に変更 (dlib fallback常設) | InsightFace重みは非商用 |
| ComposableNode化で高速化 | ComposableNode化はやらない | rclpy zero-copy未実装 — 効果ゼロ |
| 評価指標としてmAP/IDF1のみ | + DAiSEE engagement accuracy | engagement判定の良し悪しを直接測れる指標が必要 |
| Phase 1 = msg定義 | Phase 1 = 検証基盤 (msg移行は並行) | 「数値で語れる」状態を最優先で作る |

---

## 7. 用語と参照

### 7.1 主要パッケージ

- ROS4HRI: <https://github.com/ros4hri>
- hri_msgs (REP-155): <https://github.com/ros4hri/hri_msgs>
- vision_msgs: <https://github.com/ros-perception/vision_msgs>
- hri_face_detect: <https://github.com/ros4hri/hri_face_detect>
- hri_engagement: <https://github.com/ros4hri/hri_engagement>
- 6DRepNet_ros: <https://github.com/thohemp/6DRepNet_ros>

### 7.2 採用モデル

- YuNet (OpenCV Zoo): <https://github.com/opencv/opencv_zoo>
- MagFace: <https://github.com/IrvingMeng/MagFace>
- 6DRepNet: <https://github.com/thohemp/6DRepNet>
- HSEmotion-ONNX: <https://github.com/av-savchenko/hsemotion-onnx>
- MediaPipe Face Landmarker: <https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker>

### 7.3 評価データセット

- WIDER FACE: <http://shuoyang1213.me/WIDERFACE/>
- LFW: <http://vis-www.cs.umass.edu/lfw/>
- AFLW2000-3D: <http://www.cbsr.ia.ac.cn/users/jjyan/main.htm>
- AffectNet: <http://mohammadmahoor.com/affectnet/>
- DAiSEE: <https://people.iith.ac.in/vineethnb/resources/daisee/>

### 7.4 検証技術

- launch_pytest: <https://github.com/ros2/launch/tree/master/launch_pytest>
- realsense-ros (rosbag テストパターン): <https://github.com/IntelRealSense/realsense-ros/tree/ros2-development/realsense2_camera/test>
- action-ros-ci: <https://github.com/ros-tooling/action-ros-ci>
- ros2_tracing: <https://github.com/ros2/ros2_tracing>
- py-motmetrics: <https://github.com/cheind/py-motmetrics>
- TrackEval: <https://github.com/JonathonLuiten/TrackEval>

### 7.5 engagement定義の根拠

- Altuwairqi et al. Concentration Index: <https://arxiv.org/abs/1909.12913>
- Webb & Lemaignan visual social engagement (hri_engagement採用): <https://github.com/ros4hri/hri_engagement>
- DAiSEE 論文: <https://arxiv.org/abs/1609.01885>
- ViBED-Net 2025 (DAiSEE SOTA 73.43%): <https://arxiv.org/abs/2510.18016>

---

## 8. 次のアクション

1. **本書のレビューを依頼** — 特に以下の点で意思決定が必要:
   - Phase 1 のスコープ (検証基盤一式 vs 最小限から段階的に)
   - 評価データセットの優先順位 (WIDER FACE/LFW を最優先、DAiSEE は Phase 4 で間に合う)
   - 既存 `String` I/F の維持期間 (Phase 5 まで並行 vs Phase 2 で即廃止)
2. レビュー反映後、Phase 0 残タスクを実施
3. Phase 1 着手判断
