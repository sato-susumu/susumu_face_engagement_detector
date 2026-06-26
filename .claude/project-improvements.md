# 改善履歴・教訓

Phase 0 から Phase 5+ までの主要な改善と、そこから得られた一般化可能な教訓。

## Phase 0 → Phase 1: 検証可能な状態への移行

### Before
- 合成 `test_images/` を目視確認
- 「動く / 動かない」しか語れない
- 改善の優劣を数値で示せない

### After
- WIDER FACE val (3,226 画像) で AP@0.5 を取得
- LFW pairs で 1:1 verification accuracy 計測
- 全結果を `outputs/baselines/*.json` に固定スキーマで永続化
- `make eval-report` で比較表自動生成

### 教訓
- **「先進化」を主張するなら数値の前後比較が必須** — ベースライン取得を Phase 1 で最優先にしたのは正解
- 公開ベンチマーク (WIDER FACE / LFW) を使うことで再現性と業界比較性を確保
- 評価結果の JSON スキーマを早期に固定 → 後段の可視化・レポート・CI 全てがそれに乗れた

## Phase 1 → Phase 2: 検出バックエンドの近代化

### Before
- dlib HOG: WIDER FACE val AP@0.5 = **13.70%**, latency mean 232.6 ms

### After
- YuNet (OpenCV Zoo, Apache 2.0): AP@0.5 = **58.79%**, latency mean 23.4 ms
- 検出精度 4.3 倍、レイテンシ 10 倍 (log scale 必要なほど)
- ライセンスもクリーン (Apache 2.0)

### 教訓
- **古いライブラリ (face_recognition / dlib HOG) は 2018 年から実質メンテ停止** — 同等以上の代替を採用すべき
- バックエンドは抽象クラス化 (`backends/detection.py`) しておくと差し替えが ROS param 1 行
- 「商用 OK」と書ける構成 (Apache 2.0 重み) を選ぶことで採用の幅が広がる

## Phase 2 → Phase 3: モダリティ追加 (姿勢・視線・表情)

### Before
- 顔位置の中央寄り判定だけで「注視」を近似
- 表情・頭部姿勢の概念なし
- engagement は 2 値 (DETECTED / LOST + ENGAGED / DISENGAGED) のみ

### After
- `head_pose_node`: MediaPipe + solvePnP で yaw/pitch/roll
- `gaze_node`: MediaPipe Iris で 3D 視線方向ベクトル
- `expression_node`: HSEmotion ONNX で 8 感情カテゴリ + softmax 確率
- すべて CPU リアルタイム動作 (mediapipe / hsemotion)

### 教訓
- **「engagement」の名にふさわしい実装には少なくとも視線 + 表情 + 姿勢が必要**
- CPU only でも MediaPipe + HSEmotion で十分動く (GPU 不要が判明)
- GPU 必要なモデル (L2CS-Net 等) は後発の拡張として残せばよい

## Phase 3 → Phase 4: engagement 再定義

### Before
- 「顔が画面中央近くで持続時間 N 秒」 → ENGAGED
- 視線方向の概念なし、表情の影響なし

### After
- Altuwairqi Concentration Index 採用:
  ```
  raw  = EmotionWeight × GazeWeight / max(EmotionWeights)
  filt = EMA(raw, α=0.3)
  gated = filt × (HeadPoseGate ? 1 : 0.3)
  ```
- `hri_msgs/EngagementLevel` 5 値出力 (UNKNOWN/DISENGAGED/ENGAGING/ENGAGED/DISENGAGING)
- ヒステリシスで状態遷移の flapping 防止

### 教訓
- 文献に基づく計算式を採用すると「なぜそのスコア式なのか」を説明しやすい
- 5 値 + EMA + ヒステリシスは現場のロボット応用で十分な表現力
- 値域・閾値はすべて ROS param で外部化 → 環境に応じてチューニング可能

## Phase 5+: 成果物の集約と可視化

### Before
- 評価結果 JSON は `eval/baselines/` 直下
- 可視化は無し (数値表のみ)
- 古い launch/scripts/docs が混在

### After
- 成果物 `outputs/` に一本化 (baselines / figures / reports / runs)
- `make outputs` 一発で評価 + 可視化 + レポート生成
- matplotlib で 3 カテゴリ可視化 (detection backend 比較、recognition ROC、engagement 時系列)
- レポート (`outputs/reports/REPORT.md`) に図埋め込み
- レガシー (旧ノード/launch/scripts/docs/test_images) すべて削除

### 教訓
- **生成物と原本コードを明確に分離** (outputs/ ↔ eval/) するとリポジトリが小さく保てる
- 「数字 + 図 + 説明文」が揃った Markdown レポートが説得力を最大化
- レガシーは「将来読みたいかも」で残すと無限に肥大化する — git history に任せて削除

## 失敗から学んだこと

### sys.modules 汚染問題 (Phase 1)
**失敗:** レガシーテストで `sys.modules['cv2'] = MagicMock()` を実行 → pytest セッション全体に副作用 → 新規 unit テストが謎の MagicMock を受け取る。

**対策:** subprocess 起動でテストを完全隔離 + レガシーテスト本体を削除 (Phase 5+)。

**一般化:** モジュール grobal mock は使わない、または使うならテストファイル単位で完全に閉じ込める。

### setup.cfg なしで executable が見つからない (Phase 2)
**失敗:** `ros2 pkg executables` が空。`ros2 run` も "No executable found"。

**対策:** `setup.cfg` で `install_scripts=$base/lib/<pkg>` を明示。

**一般化:** ament_python パッケージは `setup.cfg` が事実上必須。テンプレートに組み込むべき。

### バックグラウンド実行を長時間ブロックして待つ
**失敗:** dlib HOG full WIDER FACE val (12分) を `while ps; do sleep 30` で待機 → ユーザーから「何待ってる?」とツッコミ。

**対策:** 同じ数値が既に取れているなら再実行不要。バックグラウンドプロセスは目的が明確な時だけ。

**一般化:** 「動く」「数値を取る」が目的なら一度成功したら次に移る。冗長な再計算は時間の浪費。

### vision_msgs/BoundingBox2D.center のネスト構造
**失敗:** `bb.center.x = 100.0` → `AttributeError`。`center` は Pose2D で `position.x` にネスト。

**対策:** `bb.center.position.x = float(cx)`。テストで実際に msg を構築・assert する contract test を書いた。

**一般化:** 外部 msg 仕様は ソース (.msg ファイル) を必ず参照。`grep -A3 BoundingBox2D /opt/ros/humble/share/.../`.

## 今後の拡張余地

### 商用構成 → 研究 SOTA への切り替え
backend abstraction はすでに用意済み。追加すべきは以下のみ:
- `backends/detection.py` に `SCRFDBackend` (InsightFace, 非商用重み)
- `backends/recognition.py` に `ArcFaceBackend` / `MagFaceBackend`
- `backends/headpose.py` に `SixDRepNetBackend` (MIT, GPU)
- `backends/gaze.py` に `L2CSNetBackend` (MIT, GPU)

### 多人物対応
現状 engagement_node は単一人物 (`default`)。`hri_person_manager` 統合で多人物 ID 解決後、`/humans/persons/<id>/...` を各人物分発行する設計に拡張可能。

### TensorRT / ONNX Runtime GPU
`onnxruntime-gpu` 入れて backend を ORT 経由にすれば 2-5x 加速。NITROS (Isaac ROS) で intra-process zero-copy も検討候補 (ただし rclpy では zero-copy 未実装、C++ 化が前提)。

### 評価データセット拡充
現状未着手:
- MPIIGaze / Gaze360 (視線推定の定量評価)
- AFLW2000-3D (頭部姿勢 MAE)
- AffectNet (表情 confusion matrix) ※要申請
- DAiSEE (engagement 4階級 accuracy) ※要申請

これらが揃えば Phase 6 として **engagement 全モダリティを公開ベンチマークで定量化** できる。
