# デバッグログ・重要記録

Phase 0-5+ で実際に踏んだ罠と解決策。新規開発時に同じ問題を繰り返さないための記録。

## Phase 5+ (2026-06): レガシー全削除と outputs/ 集約

### 問題: 旧 install/ 配下の executable が消えない
`setup.py` から旧ノード (face_engagement_node 等) を削除しても、`ros2 pkg executables` に残り続けた。

**原因:** `colcon build` は install/ にすでに置かれた entry_point を再ビルドのみでは消さない。

**対策:**
```bash
rm -rf build/susumu_face_engagement_detector install/susumu_face_engagement_detector
colcon build --packages-select susumu_face_engagement_detector
```

### 問題: ament_python の console_scripts が `bin/` に置かれて `ros2 run` から見えない
**原因:** `setup.cfg` の `install_scripts` 指定が無いと setuptools デフォルトの `bin/` に行く。

**対策:** `setup.cfg` 作成:
```ini
[develop]
script_dir=$base/lib/susumu_face_engagement_detector
[install]
install_scripts=$base/lib/susumu_face_engagement_detector
```

### 問題: matplotlib 3.5 で `Legend(..., ncols=N)` が TypeError
**原因:** `ncols` キーワードは matplotlib 3.6+ で追加。3.5 系は `ncol` (S なし)。

**対策:** `eval/visualize.py` で `ax.legend(handles=handles, ncol=len(handles))`。

---

## Phase 1 (2026-06): 評価ハーネス構築

### 問題: pytest 経由でなぜか `cv2` が MagicMock になる
**症状:** unit テスト内で `cv2.imwrite` が MagicMock を返す、`cv2.imread` も同様。素の Python では正常。

**原因:** レガシーテスト (`test/test_simple.py` 等) が import 時に `sys.modules['cv2'] = MagicMock()` を実行。pytest collection で読み込まれ、同セッション内の他テストに副作用。

**対策 (短期):** legacy テストを `test/legacy/` に隔離 + `pytest.ini` の `testpaths` で除外:
```ini
[pytest]
testpaths = test/unit test/integration
```

**対策 (Phase 5+ 恒久):** legacy テスト本体を削除。

### 問題: pytest 9 で `[tool:pytest]` セクションが効かない
**症状:** `pytest.ini` の `testpaths` が無視されてリポジトリ全体を collect。

**原因:** pytest 9.x は `pytest.ini` で `[tool:pytest]` を読まなくなった (`[tool:pytest]` は `setup.cfg` 専用)。

**対策:** `[pytest]` に変更:
```ini
[pytest]   # ← [tool:pytest] ではダメ
testpaths = test/unit test/integration
```

### 問題: integration テストで cv_bridge が MagicMock 化する
同じく sys.modules 汚染の遠回しな副作用。`rclpy + cv_bridge + 実 ROS 2 ノード起動` を pytest 内でやると mock がいたるところに。

**対策:** subprocess 経由でテストハーネスを別 Python プロセスで起動:
```python
script = tmp_path / "harness.py"
script.write_text(HARNESS_SCRIPT)
cmd = ["bash", "-c", f"source /opt/ros/humble/setup.bash && python3 {script}"]
proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
# harness は RESULT_JSON=... を stdout に出力 → 親で json.loads
```

`test/integration/test_pipeline_*.py` 全てこの方式。

---

## Phase 2 (2026-06): YuNet 統合

### 問題: vision_msgs/BoundingBox2D の center 構造
**症状:** `bb.center.x = 100.0` で `AttributeError: 'Pose2D' object has no attribute 'x'`。

**原因:** vision_msgs 4.x の `BoundingBox2D.center` は `vision_msgs/Pose2D` で、内側に `position` (Point2D) と `theta` がある。

**対策:**
```python
bb.center.position.x = float(cx)   # 内側にネスト
bb.center.position.y = float(cy)
```

### 問題: MediaPipe (0.10.21) が起動時 protobuf エラー
**症状:** `[libprotobuf ERROR] CalculatorGraphConfig: Expected identifier, got: \`

**現状:** Phase 2 検証で発見、未解決。MediaPipe `mediapipe.solutions.face_detection` の利用は控え、Face Mesh (refine_landmarks=True) のみ使用。

**回避策:** Phase 3 の head_pose_node / gaze_node は `mp.solutions.face_mesh.FaceMesh` を使い問題なく動作。

---

## Phase 0 (2025-06, 旧バージョン): 歴史的記録

旧ベースライン (dlib HOG, 統合型 face_engagement_node) の時代に観測したパフォーマンス問題:

- CPU 90%+: 高解像度画像を直接 HOG にかけていた → downsample 50% で半減
- 長時間メモリ漏れ: face_recognition のスコープ管理 → del + gc.collect
- フレームスキップ (3 フレームに 1 回処理)

これらは Phase 2+ で YuNet (CPU 1 コア 30ms) に置き換わり、もはや問題ではない。記録のみ残置。

---

## デバッグ Tips (汎用)

### ROS 2 ノードがトピックに発行しない
```bash
ros2 node info /node_name           # publisher リスト確認
ros2 topic info /topic_name -v      # subscriber 数, QoS 確認
ros2 topic hz /topic_name           # 実発行レート
ros2 topic delay /topic_name        # publisher と subscriber の時刻差 (要 header.stamp)
```

### バックエンド切替で「期待通り動かない」
1. パラメータ実値確認: `ros2 param get /node_name detection_backend`
2. backend 初期化ログ: `--ros-args --log-level debug`
3. 重みパスが存在するか: `ls -l ~/models/...`

### 評価結果が前回と違う
1. 入力データセットが同じか確認 (WIDER FACE val の md5)
2. 同じ backend / config か `outputs/baselines/*.json` の `config` フィールド比較
3. matplotlib / cv2 / mediapipe のバージョン: `pip3 list | grep -iE "(matplotlib|opencv|mediapipe)"`
