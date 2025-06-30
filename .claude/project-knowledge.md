# 技術知見・パターン

## ROS2開発パターン

### ノード初期化パターン
```python
def __init__(self):
    super().__init__('node_name')
    self.declare_parameter('param_name', default_value)
    self.param = self.get_parameter('param_name').get_parameter_value().string_value
```

### コールバック処理パターン
```python
def callback(self, msg):
    try:
        # 処理ロジック
        self.publisher.publish(result_msg)
    except Exception as e:
        self.get_logger().error(f'Error: {e}')
```

## 顔認識技術知見

### 顔特徴量処理
- **128次元ベクトル** - face_recognitionライブラリの標準
- **CSV形式保存** - `','.join(map(str, encoding))`
- **ユークリッド距離** - 顔マッチング判定用

### モデル選択指針
- **HOGモデル** - CPU効率重視、リアルタイム用
- **CNNモデル** - 精度重視、GPU使用時

### 顔検出最適化
```python
# 画像サイズ調整による高速化
small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
face_locations = face_recognition.face_locations(small_frame, model='hog')
# 座標を元サイズにスケール
face_locations = [(top*4, right*4, bottom*4, left*4) for (top, right, bottom, left) in face_locations]
```

## パフォーマンス最適化

### メモリ管理
```python
# 不要な変数の削除
del large_array

# ガベージコレクション強制実行
import gc
gc.collect()
```

### フレーム処理効率化
```python
# フレームスキップによる負荷軽減
self.frame_count += 1
if self.frame_count % 3 != 0:  # 3フレームに1回処理
    return
```

## テスト戦略

### モック使用パターン
```python
@patch('rclpy.node.Node.create_publisher')
@patch('rclpy.node.Node.create_subscription')
def test_node_initialization(self, mock_sub, mock_pub):
    node = TestNode()
    assert node is not None
```

### 統合テストパターン
```python
def test_pipeline_integration():
    # 複数ノード間の通信テスト
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
```

## エラーハンドリング

### ROS2例外処理
```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### OpenCV例外処理
```python
try:
    frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
except CvBridgeError as e:
    self.get_logger().error(f'CV Bridge error: {e}')
    return
```

## 設定管理

### パラメータファイル構造
```yaml
/**:
  ros__parameters:
    param_name: value
    nested_param:
      sub_param: value
```

### 動的パラメータ変更
```python
self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'threshold':
            self.threshold = param.value
    return SetParametersResult(successful=True)
```

## デバッグ技法

### ログレベル設定
```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### トピック監視
```bash
ros2 topic echo /topic_name
ros2 topic hz /topic_name
```

### ノード情報確認
```bash
ros2 node info /node_name
ros2 param list /node_name
```

## ベストプラクティス

### コード品質
- **型ヒント使用** - `def process(self, data: str) -> bool:`
- **docstring記述** - 関数・クラスの説明
- **ログ出力統一** - self.get_logger().info/warn/error

### パフォーマンス
- **不要な計算回避** - キャッシュ活用
- **メモリリーク防止** - 適切なリソース解放
- **CPU負荷分散** - フレームスキップ等

### 保守性
- **設定外部化** - パラメータファイル使用
- **モジュール分離** - 単一責任原則
- **テスト充実** - 単体・統合テスト