# 技術知見・パターン

このパッケージで実証済の設計・実装パターンの記録。新しいノード/バックエンドを追加するときの参考。

## ノード設計

### バックエンド抽象化パターン

各機能 (検出/認識/姿勢/視線/表情) は **共通抽象クラス + 複数バックエンド** で実装。`susumu_face_engagement_detector/backends/*.py` 参照。

```python
class DetectionBackend:
    name: str = "abstract"
    def detect(self, bgr): raise NotImplementedError

class YuNetBackend(DetectionBackend):
    name = "yunet"
    def __init__(self, model_path, score_threshold=0.5, nms_threshold=0.3):
        self._detector = cv2.FaceDetectorYN_create(...)
    def detect(self, bgr):
        ...

_BACKENDS = {"dlib_hog": DlibHogBackend, "yunet": YuNetBackend, ...}

def make_backend(name: str, **kwargs):
    # 未対応のkwargsは sweep して落とす — ノードからは全 backend に
    # 同じconfig dict を渡せる
    import inspect
    sig = inspect.signature(_BACKENDS[name].__init__)
    accepted = {p for p in sig.parameters if p != "self"}
    cleaned = {k: v for k, v in kwargs.items() if k in accepted and v is not None}
    return _BACKENDS[name](**cleaned)
```

利点: ノード本体は backend に無関係、テスト可能 (stub backend を差し込める)、ROS param 切替で全 backend をカバーできる。

### ノード初期化での backend 構築

```python
class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self._declare_params()
        backend_name = self.get_parameter('detection_backend').value or 'dlib_hog'
        self._detector = make_backend(
            backend_name,
            model_path=self.get_parameter('model_path').value,
            score_threshold=self.get_parameter('score_threshold').value,
            nms_threshold=self.get_parameter('nms_threshold').value,
        )
```

### コールバック処理

```python
def _on_image(self, msg: Image) -> None:
    try:
        bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception as e:
        self.get_logger().warn(f'cv_bridge error: {e}')
        return
    result = self._backend.detect(bgr)
    # vision_msgs/Detection2DArray を組み立て発行
    ...
```

## メッセージ標準

### vision_msgs/BoundingBox2D の落とし穴

`BoundingBox2D.center` は `vision_msgs/Pose2D` で、座標は **内側の `.position.x/y` にネスト**:

```python
bb = BoundingBox2D()
bb.center.position.x = float(cx)   # ✕ bb.center.x ではない
bb.center.position.y = float(cy)
bb.size_x = float(w)
bb.size_y = float(h)
```

### hri_msgs/EngagementLevel の定数

5値定義 (`hri_msgs/EngagementLevel.msg`):

| 値 | 定数 | 意味 |
|---|---|---|
| 0 | UNKNOWN | 情報なし |
| 1 | DISENGAGED | 注意していない |
| 2 | ENGAGING | 注意し始めている |
| 3 | ENGAGED | 完全に注意中 |
| 4 | DISENGAGING | 注意を逸らし始めている |

```python
from hri_msgs.msg import EngagementLevel
msg = EngagementLevel()
msg.header.stamp = self.get_clock().now().to_msg()
msg.level = EngagementLevel.ENGAGED  # 整数 3
```

### ROS4HRI トピック命名規約 (REP-155)

```
/humans/faces/<id>/<property>
/humans/persons/<id>/<property>
/humans/faces/tracked    (hri_msgs/IdsList)
```

## 評価ハーネス

### 評価結果の JSON スキーマ (安定化済)

`outputs/baselines/*.json` の構造を破壊的に変えない (レポート/CI が依存):

```python
{
  "backend": "yunet",
  "label": "v1 YuNet (WIDER FACE val, full)",
  "config": {...},
  "dataset": "wider_face_val",
  "iou_threshold": 0.5,
  "overall": {"ap": 0.5879, "num_gt": ..., "num_pred": ...},
  "subsets": {"easy": {"ap": ...}, "medium": ..., "hard": ...},
  "latency_ms": {"mean": ..., "p50": ..., "p95": ..., "p99": ...},
  "num_images": 3226,
  "elapsed_seconds": 83.7
}
```

### Detection AP の計算

PASCAL VOC 2010+ all-points 補間 (WIDER FACE 公式評価と互換)。`eval/metrics/detection.py` 参照。

### Recognition Accuracy

コサイン類似度の閾値スイープで最良精度を採用。`eval/metrics/recognition.py`。LFW で 98.50% / AUC 0.9974 (dlib 128-D, 1,000 pairs)。

## エンゲージメント定義

Altuwairqi らの Concentration Index 式 (`susumu_face_engagement_detector/engagement.py`):

```
EmotionWeight = {neutral: 0.9, happy: 0.7, surprise: 0.6, sad: 0.3, ...}
GazeWeight    = 1.0 (≤15°) | 0.5 (≤30°) | 0.0 (>30°)
HeadPoseGate  = (|yaw|<15°) AND (|pitch|<10°)

raw   = EmotionWeight × GazeWeight / max(EmotionWeights)
filt  = EMA(raw, α=0.3)
gated = filt if HeadPoseGate else filt × 0.3
```

5状態遷移にはヒステリシス。詳細 → `engagement._classify`。

## テスト戦略

### sys.modules 汚染回避

過去のレガシーテストが `sys.modules['cv2'] = MagicMock()` を実行してpytestセッション全体に副作用を持っていた。対策:

1. レガシーテストは削除済 (Phase 5+)。
2. ROS 2 ノードを走らせる integration テストは **subprocess** で隔離 (`test/integration/test_pipeline_*.py`):

```python
HARNESS_SCRIPT = textwrap.dedent("""
    import rclpy
    from cv_bridge import CvBridge  # mock 化されない
    ...
""")
def test_xxx(tmp_path):
    script = tmp_path / "harness.py"
    script.write_text(HARNESS_SCRIPT)
    cmd = ["bash", "-c", f"source {ROS_SETUP} && python3 {script}"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    # RESULT_JSON=... を harness 側で stdout に印字、親で json.loads
```

### pytest 設定 (pytest.ini)

```ini
[pytest]
testpaths = test/unit test/integration  # legacy/ を除外
python_files = test_*.py
```

### unit テストの基本パターン

```python
import pytest
from eval.metrics.recognition import Pair, best_threshold_accuracy

def test_accuracy_perfectly_separable():
    pairs = [Pair(a=v, b=v, same=True) for v in ...]
    pairs += [Pair(a=a, b=-a, same=False) for ...]
    res = best_threshold_accuracy(pairs)
    assert res["accuracy"] == 1.0
```

## CI / GitHub Actions

`.github/workflows/ci.yml` で `setup-ros@v0.7` + `action-ros-ci@v0.4`。2 ジョブ:

1. `unit-and-contract` — colcon build + 全 pytest
2. `detection-smoke` — 合成 WIDER FACE-like 画像で評価ランナーの schema 検証

## ビルド / インストールの落とし穴

### setup.cfg が無いと console_scripts は `bin/` に行く

ament_python は `setup.cfg` の `install_scripts=$base/lib/<pkg>` 指定が無いと `bin/` にインストールされ、`ros2 run` から見えない:

```ini
[develop]
script_dir=$base/lib/susumu_face_engagement_detector
[install]
install_scripts=$base/lib/susumu_face_engagement_detector
```

### 古い install ディレクトリ残存

`setup.py` から entry_point を消しても install/ 配下の旧 executable は残る。新しいノードを追加・削除したら:

```bash
rm -rf build/susumu_face_engagement_detector install/susumu_face_engagement_detector
colcon build --packages-select susumu_face_engagement_detector
```

## 可視化 (matplotlib)

`eval/visualize.py` で 3 サブコマンド (detection / recognition / engagement)。

- ヘッドレス: `matplotlib.use("Agg")` 必須
- レイアウト崩れ防止: `fig.tight_layout()` + `dpi=120`
- matplotlib 3.5 系では `Legend(..., ncols=N)` は **`ncol=N`** (S なし) を使うこと。3.6+ で `ncols` が入る

## モデル重みのライセンス境界

| ライブラリ | コード | 学習済み重み | 商用可否 |
|---|---|---|---|
| dlib face_recognition | Boost SL | パブリックドメイン | ✓ |
| YuNet (OpenCV Zoo) | Apache 2.0 | Apache 2.0 | ✓ |
| MediaPipe | Apache 2.0 | Apache 2.0 | ✓ |
| HSEmotion | Apache 2.0 | Apache 2.0 | ✓ |
| InsightFace (SCRFD/ArcFace) | MIT | **非商用** | ✗ 商用は別契約 |
| Ultralytics YOLO v8-v12 | AGPL-3.0 | AGPL-3.0 | ✗ Enterprise License 必要 |

非商用 backend (InsightFace / Ultralytics) を採用したい場合は、**研究モードでのみ使用** とドキュメントに明記すること。

## 設定 (config/*.yaml)

```yaml
/**:
  ros__parameters:
    detection_backend: yunet
    model_path: /home/user/models/face_detection/face_detection_yunet_2023mar.onnx
    score_threshold: 0.5
    nms_threshold: 0.3
```

`launch/engagement_pipeline.launch.py` から `parameters=[...]` で渡す。動的変更は `ros2 param set` で可能。

## デバッグ

```bash
# トピック発行 hz / 遅延
ros2 topic hz /face_detections_vision
ros2 topic delay /face_detections_vision

# ノード接続関係
rqt_graph

# ログレベル
ros2 run susumu_face_engagement_detector face_detection_node --ros-args --log-level debug
```
