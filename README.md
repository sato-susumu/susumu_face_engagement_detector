# Susumu Face Engagement Detector

カメラ映像から **顔検出 → 認識 → 頭部姿勢 → 視線推定 → 表情 → エンゲージメント** までを ROS 2 (Humble) で行うパッケージです。視線推定は OpenVINO モデルがある時だけ有効にする optional 機能です。

出力は [ROS4HRI / REP-155](https://github.com/ros-infrastructure/rep/blob/master/rep-0155.rst) と [vision_msgs](https://github.com/ros-perception/vision_msgs) の標準メッセージに準拠。すべての評価結果と図は [`outputs/`](outputs/) 配下に自動集約され、`make outputs` で再現可能です。

---

## ハイライト

| | dlib HOG (v0) | **YuNet (v1)** | 改善 |
|---|---:|---:|---:|
| 顔検出 AP@0.5 (WIDER FACE val 3,226枚) | 13.70% | **58.79%** | **×4.3** |
| 検出レイテンシ (mean / p95) | 232.6 / 406.8 ms | **23.4 / 63.9 ms** | **×10** |
| 認識 1:1 (LFW pairs test) | **98.50%** acc, AUC 0.9974, 4.1 ms / pair (dlib 128-D) |||
| ライセンス | コード Boost SL / 重みパブリックドメイン | **Apache 2.0** (商用可) ||

![detection backend comparison](outputs/figures/detection/backend_comparison.png)

完全な数値レポート (図入り) → [`outputs/reports/REPORT.md`](outputs/reports/REPORT.md)

---

## アーキテクチャ

```
camera/color/image_raw
    │
    ├─→ face_detection_node ─── vision_msgs/Detection2DArray
    │      backend: dlib_hog | dlib_cnn | yunet
    │
    ├─→ face_recognition_node ─ hri_msgs/IdsList
    │      backend: dlib_128d
    │
    ├─→ head_pose_node ──────── geometry_msgs/PoseStamped
    │      backend: mediapipe_pnp
    │
    ├─→ gaze_node ───────────── geometry_msgs/Vector3Stamped
    │      backend: openvino_adas (optional)
    │
    ├─→ expression_node ─────── hri_msgs/Expression
    │      backend: hsemotion (Apache 2.0)
    │
    └─→ engagement_node ─────── hri_msgs/EngagementLevel  (5値 + EMA + ヒステリシス)
           Altuwairqi Concentration Index
```

詳細な変革経緯と設計判断: [`docs/REVAMP_PLAN.md`](docs/REVAMP_PLAN.md)

---

## クイックスタート

### 1. 依存関係

```bash
sudo apt install \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-hri-msgs \
    ros-humble-rosbag2-storage-mcap

pip3 install --user "protobuf<5,>=4.25.3" mediapipe hsemotion-onnx face_recognition opencv-contrib-python
```

OpenVINO 視線推定を使う場合だけ、追加で OpenVINO とモデルを用意します。

```bash
pip3 install --user openvino

mkdir -p ~/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32
wget https://storage.openvinotoolkit.org/repositories/open_model_zoo/2021.4/models_bin/1/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml \
    -O ~/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml
wget https://storage.openvinotoolkit.org/repositories/open_model_zoo/2021.4/models_bin/1/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.bin \
    -O ~/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.bin
```

### 2. モデル重みの取得 (どちらも Apache 2.0)

```bash
# YuNet 顔検出
mkdir -p ~/models/face_detection
wget https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx \
    -O ~/models/face_detection/face_detection_yunet_2023mar.onnx

# HSEmotion 表情 (urllib バグ回避のため事前取得)
mkdir -p ~/.hsemotion
wget https://github.com/HSE-asavchenko/face-emotion-recognition/raw/main/models/affectnet_emotions/onnx/enet_b0_8_best_afew.onnx \
    -O ~/.hsemotion/enet_b0_8_best_afew.onnx
```

### 3. ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_face_engagement_detector
source install/setup.bash
```

### 4. 起動

```bash
# フルパイプライン (YuNet + head_pose + expression + engagement)
ros2 launch susumu_face_engagement_detector engagement_pipeline.launch.py

# OpenVINO 視線推定も起動する場合
ros2 launch susumu_face_engagement_detector engagement_pipeline.launch.py \
    enable_gaze:=true \
    gaze_model_path:=$HOME/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml

# トピック確認
ros2 topic echo /humans/persons/default/engagement_status
ros2 topic echo /humans/persons/default/engagement_score
```

個別ノード起動例:

```bash
ros2 run susumu_face_engagement_detector face_detection_node \
    --ros-args -p detection_backend:=yunet \
               -p model_path:=$HOME/models/face_detection/face_detection_yunet_2023mar.onnx

ros2 run susumu_face_engagement_detector head_pose_node
ros2 run susumu_face_engagement_detector gaze_node \
    --ros-args -p gaze_model_path:=$HOME/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml
ros2 run susumu_face_engagement_detector expression_node
ros2 run susumu_face_engagement_detector engagement_node
```

### 5. 動画ファイルで機能を確認する

ROS 2 カメラ入力を用意しなくても、通常の動画ファイルから **顔検出 / 簡易人物 ID / 頭部姿勢 / 視線 / 表情 / エンゲージメント** を重畳した MP4 を生成できます。OpenVINO gaze model が未配置の場合、視線だけ無効化して処理を続けます。

```bash
# public domain のサンプル動画を自動取得し、顔が出る 7 秒地点から 20 秒を注釈付き MP4 に変換
make video-demo

# 複数人物が同じ画面に長く映る 90 秒デモを生成
make video-demo-multi

# 正解ユーザーID付き ChokePoint 顔IDデモを生成 (CC BY-NC 4.0、非商用検証用)
make chokepoint-gt-demo

# 自前の動画を使う場合
make video-demo VIDEO_DEMO_SOURCE=/path/to/input.mp4 VIDEO_DEMO_START=0 VIDEO_DEMO_SECONDS=0

# 顔検出の信頼度閾値。0.8 未満は顔として扱わず、描画・認識・表情推定に回しません
make video-demo VIDEO_DEMO_SCORE_THRESHOLD=0.8

# 人物IDを付ける信頼度閾値。顔としては描画しても、0.9 未満は user_* を付けません
make video-demo VIDEO_DEMO_IDENTITY_SCORE_THRESHOLD=0.90

# 同一人物の ID が割れる場合は embedding 閾値を少し緩める
make video-demo VIDEO_DEMO_MATCH_TOLERANCE=0.75

# 1位候補と直前IDの距離差が小さい場合は直前IDを維持する
make video-demo VIDEO_DEMO_IDENTITY_MARGIN=0.15

# 小さい顔にも人物 ID を付けたい場合は下げる。特徴量が不安定になり誤認は増えます
make video-demo VIDEO_DEMO_MIN_IDENTITY_FACE_PX=80

# 視線推定を明示的に無効化する場合
make video-demo VIDEO_DEMO_NO_GAZE=1
```

人物 ID は、顔検出信頼度・顔サイズ・頭部姿勢が ID 付与条件を満たし、かつ顔特徴量 embedding の距離が閾値内のときだけ既存 `user_*` に割り当てます。同一フレーム内で同じ `user_*` を複数の顔へ割り当てることはありません。距離が近い候補が複数ある場合は、`VIDEO_DEMO_IDENTITY_MARGIN` の範囲で直前に表示していたトラック ID を優先します。特徴量が取れない顔や、サイズ不足・低信頼度・横向きなどで特徴量が不安定な顔には人物 ID を付けず、`face` / `unidentified` として扱います。

成果物:

```text
outputs/runs/video_demo/
├── there_isnt_just_one_face_to_breast_cancer.webm  # デフォルト入力動画
├── annotated_engagement_demo.mp4                   # 注釈付き出力動画
├── nasa_astronauts_discuss_life_in_space_multi_person.webm  # 複数人物の長尺入力動画
├── annotated_multi_person_demo.mp4                 # 複数人物の注釈付き出力動画
└── annotated_engagement_demo.mp4.json              # 実行メタデータ
```

デフォルト入力は Wikimedia Commons の [There Isn’t Just One Face to Breast Cancer](https://commons.wikimedia.org/wiki/File:There_Isn%E2%80%99t_Just_One_Face_to_Breast_Cancer.webm) です。CDC/HHS 制作の米国連邦政府職務著作として public domain と表示されているため、複製して検証入力に使えます。

複数人物デモ入力は Wikimedia Commons の [NASA Astronauts Discuss Life In Space With Fox News](https://commons.wikimedia.org/wiki/File:NASA_Astronauts_Discuss_Life_In_Space_With_Fox_News_%28iss073m262461622%29.webm) です。NASA Johnson Space Center 制作の public domain 動画で、同じ画面に複数の人物がはっきり映ります。`make video-demo-multi` はこの長尺入力の 35 秒地点から 90 秒を 1920px 幅の注釈付き MP4 にします。

正解データ付きデモは [ChokePoint Dataset](https://zenodo.org/records/815657) を使います。人物ID別の顔 crop 連番から、GT `person_id` と認識結果を並べた MP4 と JSON を生成します。ChokePoint は **CC BY-NC 4.0** なので、非商用の検証・研究用途として扱ってください。
`make chokepoint-gt-demo CHOKEPOINT_GT_IDENTITY_MARGIN=0.15` のように指定すると、GT デモでも直前表示 ID を優先する margin を調整できます。

内蔵の複数人物サンプルを直接 CLI で実行する場合:

```bash
python3 -m eval.video_demo \
    --sample multi_person \
    --output outputs/runs/video_demo/annotated.mp4 \
    --start-seconds 0 \
    --max-seconds 30 \
    --score-threshold 0.8 \
    --identity-score-threshold 0.9 \
    --match-tolerance 0.75 \
    --identity-margin 0.15 \
    --gaze-model-path $HOME/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml \
    --min-identity-face-px 110 \
    --model-path $HOME/models/face_detection/face_detection_yunet_2023mar.onnx
```

自前動画を直接 CLI で実行する場合は `--input /path/to/input.mp4` を指定します。

---

## 検証・評価・可視化 — すべて `outputs/` に集約

### 一発で全部走る

```bash
make outputs
```

これで `outputs/` 配下に baselines (JSON) + figures (PNG) + reports/REPORT.md が揃います。実行内容:

1. dlib HOG smoke (100画像) → `outputs/baselines/v0_dlib_hog_smoke_n100.json`
2. YuNet full (WIDER FACE val 3,226画像, 約1.5分) → `outputs/baselines/v1_yunet_wider_val.json`
3. dlib 128-D recognition (LFW pairs 1,000) → `outputs/baselines/v0_dlib_128d_lfw.json`
4. 3カテゴリの可視化 → `outputs/figures/{detection,recognition,engagement}/`
5. 図埋め込みレポート → `outputs/reports/REPORT.md`

### 個別実行

```bash
# 評価
make eval-detection-baseline    # dlib HOG full WIDER FACE val   (~12 min)
make eval-detection-smoke       # dlib HOG 100枚                 (~30s)
make eval-detection-yunet       # YuNet full WIDER FACE val      (~1.5 min)
make eval-recognition-baseline  # dlib 128-D LFW pairs test      (~30s)

# 可視化
make visualize-detection        # AP/latency棒グラフ + オーバーレイ画像
make visualize-recognition      # ROC + コサイン類似度分布
make visualize-engagement       # 24秒シナリオ engagement 時系列

# レポート (図埋め込み)
make eval-report

# 全消去 (.gitignore/.gitkeep/README は残す)
make clean-outputs
```

### 生成される図

#### 検出 (`outputs/figures/detection/`)

`backend_comparison.png` — backend ごとの AP / latency 棒グラフ:

![backend comparison](outputs/figures/detection/backend_comparison.png)

`overlay_*.png` — WIDER FACE 画像上に各 backend の検出結果を横並びオーバーレイ:

![overlay sample](outputs/figures/detection/overlay_0_Parade_Parade_0_12.png)

#### 認識 (`outputs/figures/recognition/`)

| 種類 | サンプル |
|---|---|
| 同人物 / 別人物のコサイン類似度分布 | ![score distribution](outputs/figures/recognition/score_distribution_dlib_128d.png) |
| 1:1 verification ROC + AUC | ![ROC](outputs/figures/recognition/roc_dlib_128d.png) |

#### エンゲージメント (`outputs/figures/engagement/`)

24 秒の合成シナリオ (UNKNOWN → ENGAGED → DISENGAGED → ENGAGED → DISENGAGING) の時系列:

![engagement timeline](outputs/figures/engagement/engagement_timeline.png)

成果物保存場所の詳細 → [`outputs/README.md`](outputs/README.md)
評価ハーネスの拡張方法 → [`eval/README.md`](eval/README.md)

---

## メッセージ I/F

### 標準 (推奨)

| トピック | 型 | 出所 |
|---|---|---|
| `/face_detections_vision` | `vision_msgs/Detection2DArray` | ros-perception |
| `/humans/faces/tracked` | `hri_msgs/IdsList` | REP-155 |
| `/humans/faces/head_pose` | `geometry_msgs/PoseStamped` | 標準 |
| `/humans/faces/gaze` | `geometry_msgs/Vector3Stamped` | 標準 |
| `/humans/faces/expression` | `hri_msgs/Expression` | REP-155 |
| `/humans/persons/<id>/engagement_status` | `hri_msgs/EngagementLevel` (5値) | REP-155 |
| `/humans/persons/<id>/engagement_score` | `std_msgs/Float32` (連続値) | 補助 |

### 旧 String I/F (後方互換、将来削除予定)

`/face_detections`, `/face_identities` (パイプ区切り `std_msgs/String`)、`/engagement_event` (ENGAGED/DISENGAGED 文字列)。Phase 1 以前の下流ノードへの互換のため残置。新規開発では上記 標準 I/F を使用してください。

---

## エンゲージメント定義

`engagement_node` は Altuwairqi らの Concentration Index に基づくスコアリングを採用:

```
EmotionWeight = {neutral: 0.9, happy: 0.7, surprise: 0.6, sad: 0.3, ...}
HeadPoseGate  = (|yaw|<15°) AND (|pitch|<10°)  → pass

raw   = EmotionWeight / max(EmotionWeights)
filt  = EMA(raw, α=0.3)
gated = filt if HeadPoseGate else filt × 0.3

EngagementLevel (hri_msgs/EngagementLevel):
  ≥0.66 持続       → ENGAGED       (3)
  ≥0.33            → ENGAGING      (2)
  ENGAGED から低下 → DISENGAGING   (4)
  <0.10 持続       → DISENGAGED    (1)
  入力欠落         → UNKNOWN       (0)
```

すべての閾値・EMA係数・ヒステリシス回数は `engagement_node` の ROS param で調整可能。式の根拠と意思決定経緯は [docs/REVAMP_PLAN.md §2.5](docs/REVAMP_PLAN.md)。
OpenVINO 視線推定は `/humans/faces/gaze` と動画注釈に出力しますが、現時点では engagement score の入力には使っていません。

---

## ユースケース別の構成

### 商用 (ライセンス完全クリーン)

すべて Apache 2.0 / MIT / Boost SL の組合せ:

| 機能 | Backend | ライセンス |
|---|---|---|
| 検出 | `yunet` | Apache 2.0 (重み含む) |
| 認識 | `dlib_128d` | Boost SL + パブリックドメイン重み |
| 頭部姿勢 | `mediapipe_pnp` | Apache 2.0 |
| 視線 | `openvino_adas` | Apache 2.0 (Open Model Zoo、optional) |
| 表情 | `hsemotion` | Apache 2.0 (重み含む) |

### 研究 / 非商用 SOTA

差し替え候補 (重みが研究用のみ):

- 検出: SCRFD-10G (InsightFace) → WIDER Hard AP 83.05%
- 認識: ArcFace R100 / AdaFace → IJB-C(1e-4) 97%+

backend 切替は ROS param のみ — コード変更不要。

---

## テスト

```bash
make test          # unit + integration
make test-unit     # 単体のみ
```

統合テストは subprocess 起動でROS 2状態を隔離します。CI は `.github/workflows/ci.yml` で `setup-ros@v0.7` + `action-ros-ci@v0.4` を経由した自動ビルド・テスト。

---

## ドキュメント

| ドキュメント | 内容 |
|---|---|
| [docs/REVAMP_PLAN.md](docs/REVAMP_PLAN.md) | 根本変革計画 (Phase 0–5、技術選定根拠、リスク) |
| [outputs/README.md](outputs/README.md) | 成果物 (JSON / 図 / レポート) の保存先構成 |
| [outputs/reports/REPORT.md](outputs/reports/REPORT.md) | 現在のベースライン数値 + 図 |
| [eval/README.md](eval/README.md) | 評価ハーネス操作手順・拡張方法 |

---

## ライセンス

Apache License 2.0 (本パッケージのコード)。各バックエンドの**重み**ライセンスは個別に確認のこと — 「商用」構成と「研究」構成の区別は上記参照。

---

## 開発履歴

- **Phase 5+**: `outputs/` への成果物集約、可視化 (matplotlib) 自動生成、レポートへの図埋め込み、レガシー一掃 (旧ノード/launch/scripts/docs/test_images すべて削除)
- **Phase 4**: `engagement_node` 再設計 (Concentration Index + 5値 EngagementLevel)
- **Phase 3**: `head_pose_node` / `expression_node` 新設
- **Phase 2**: 検出 YuNet 化 (AP 4.3 倍)、`face_recognition_node` IdsList 並行発行
- **Phase 1**: 評価ハーネス + WIDER FACE / LFW ベースライン取得 + `vision_msgs` 並行発行
- **Phase 0**: 設計レビュー、3 並列技術調査、`docs/REVAMP_PLAN.md`
