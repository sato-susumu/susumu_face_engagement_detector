# デバッグログ・重要記録

## 重要な問題解決記録

### 2025年6月29日 - 初期開発完了
**問題:** 統合型ノードの保守性と拡張性の課題

**解決策:** マルチノードアーキテクチャへの分割
- 責任分離によるデバッグ容易化
- 個別テストによる品質向上
- モジュール化による再利用性向上

**学習:** 大規模なリファクタリングでも段階的アプローチが有効

## パフォーマンス調整記録

### 顔検出処理の最適化
**症状:** CPU使用率90%以上、リアルタイム処理困難

**調査結果:**
- 高解像度画像の直接処理が原因
- CNNモデルの計算コスト高

**対策:**
1. 画像を1/4サイズに縮小
2. HOGモデルをデフォルト採用
3. 結果座標のスケール調整

**結果:** CPU使用率50%削減、30fps達成

### メモリ使用量の改善
**症状:** 長時間実行でメモリ使用量増加

**原因特定:**
```python
# 問題のあるコード
encodings = face_recognition.face_encodings(frame)
# frameオブジェクトが解放されない
```

**修正:**
```python
# 改善後
try:
    encodings = face_recognition.face_encodings(frame)
    # 処理
finally:
    del frame
    gc.collect()
```

## 重要なバグ修正

### トピック通信の型エラー
**エラー:** `AttributeError: 'str' object has no attribute 'data'`

**原因:** メッセージ型の混在
```python
# 間違った実装
msg = String()
msg = "face_data"  # 直接代入
```

**修正:**
```python
# 正しい実装
msg = String()
msg.data = "face_data"  # dataフィールドに代入
```

### 顔認識の誤マッチング
**問題:** 異なる人物を同一人物として認識

**調査:** match_tolerance値が適切でない
- 0.3: 厳しすぎて同一人物を認識できない
- 0.8: 緩すぎて誤認識が多発

**最適化:** 0.6に設定、環境に応じて調整可能

## テスト関連の問題

### 非同期処理のテスト失敗
**症状:** 統合テストが不定期に失敗

**原因:** タイミング依存の処理
```python
# 問題のあるテスト
node.start()
result = node.get_result()  # まだ処理が完了していない
```

**解決:**
```python
# 改善後
node.start()
time.sleep(0.1)  # 処理完了待ち
result = node.get_result()
```

### モックの不適切な使用
**問題:** 実際の動作と異なる結果

**学習:** モックは外部依存のみに限定
- OpenCVの画像処理: 実際のデータ使用
- ROS2の通信: モック使用
- face_recognitionライブラリ: 制限的モック

## 設定関連の注意事項

### known_facesディレクトリ
**注意:** 空ディレクトリだと認識処理でエラー

**対策:** 
1. 最低1つの参照画像を配置
2. エラーハンドリングで空ディレクトリを検出
3. ログで適切な警告を出力

### パラメータファイルの構文
**エラー:** YAML構文エラーでノード起動失敗

**頻出問題:**
```yaml
# 間違い
parameters:
  param: value

# 正しい
/**:
  ros__parameters:
    param: value
```

## 環境依存の問題

### Ubuntu 20.04での動作
**問題:** face_recognitionライブラリのインストール失敗

**解決:** システム依存関係の事前インストール
```bash
sudo apt install python3-dev libopenblas-dev liblapack-dev
pip install face_recognition
```

### ROS2バージョン差異
**注意:** Galactic vs Humbleの違い
- パラメータ取得方法が異なる
- ログ機能のAPI変更
- ComposableNodeの実装差異

## デバッグ技法の記録

### 効果的なログ出力
```python
# 情報量の適切なログ
self.get_logger().info(f'Detected {len(faces)} faces')
self.get_logger().debug(f'Face locations: {face_locations}')
self.get_logger().warn(f'No faces detected for {self.no_face_count} frames')
```

### トピック通信の監視
```bash
# リアルタイム監視
ros2 topic echo /face_detections | grep -E "(face_idx|center)"

# 統計情報
ros2 topic hz /face_detections --window 100
```

### プロファイリング
```python
import cProfile
import pstats

def profile_face_detection():
    cProfile.run('face_detection_process()', 'profile_stats')
    stats = pstats.Stats('profile_stats')
    stats.sort_stats('cumtime').print_stats(10)
```

## 今後の注意点

### スケーラビリティ
- 複数カメラ対応時のメモリ使用量
- 並列処理時のリソース競合
- 大量の既知顔データ処理

### セキュリティ
- 顔画像データの適切な管理
- プライバシー保護機能の実装
- ログ出力での個人情報除外

### 互換性
- OpenCVバージョン更新への対応
- Python 3.11以降での動作確認
- 新しいROS2バージョンへの対応

## 緊急時対応手順

### ノード停止時
1. プロセス確認: `ps aux | grep face_`
2. 強制終了: `pkill -f face_`
3. 再起動: `ros2 launch susumu_face_engagement_detector simple_launch.py`

### 高CPU使用率時
1. 検出モード変更: `detection_model: hog`
2. フレームスキップ有効化
3. 画像解像度確認

### メモリリーク疑い時
1. メモリ使用量監視: `htop`
2. ガベージコレクション強制実行
3. ノード再起動による回復