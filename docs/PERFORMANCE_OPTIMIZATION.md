# Face Detection Performance Optimization

CPU負荷軽減機能の詳細説明

## 🎯 概要

顔検出ノード（`face_detection_node.py`）にCPU負荷軽減機能を実装しました。従来は全てのカメラフレームを処理していましたが、以下の最適化により大幅な負荷軽減を実現できます。

## 🔧 最適化機能

### 1. 最大FPS制限 (`max_fps`)
- **機能**: 処理頻度の上限を設定
- **設定**: `max_fps: 5.0` → 最大5FPSで処理
- **効果**: 高FPSカメラでもCPU負荷を制限
- **デフォルト**: `5.0` FPS

### 2. 画像縮小処理 (`downsample_factor`)
- **機能**: 顔検出時に画像を縮小
- **設定**: `downsample_factor: 0.5` → 50%に縮小
- **効果**: 処理する画素数を削減
- **デフォルト**: `0.5` (50%縮小)

## 📊 性能プロファイル

### 高性能モード
```yaml
max_fps: 0.0               # FPS制限なし
downsample_factor: 1.0     # 原寸処理
```
- **CPU負荷**: 最大
- **検出精度**: 最高
- **遅延**: 最小

### バランスモード（推奨）
```yaml
max_fps: 10.0              # 最大10FPS
downsample_factor: 0.7     # 70%縮小
```
- **CPU負荷**: 中程度
- **検出精度**: 高
- **遅延**: 低

### 省電力モード
```yaml
max_fps: 5.0               # 最大5FPS
downsample_factor: 0.5     # 50%縮小
```
- **CPU負荷**: 低
- **検出精度**: 中程度
- **遅延**: 中程度


## 🚀 使用方法

### 1. パラメータファイルを使用
```bash
# 設定ファイル使用
ros2 run susumu_face_engagement_detector face_detection_node \
  --ros-args --params-file config/face_detection_performance.yaml
```

### 2. 起動時パラメータ指定
```bash
# 省電力モード
ros2 run susumu_face_engagement_detector face_detection_node \
  --ros-args \
  -p max_fps:=3.0 \
  -p downsample_factor:=0.5
```

### 3. ランチファイル使用
```bash
# 性能最適化版ランチファイル
ros2 launch susumu_face_engagement_detector face_detection_performance.launch.py
```

## 📈 性能測定

### 測定ツール
```bash
# 簡易性能測定
python scripts/simple_performance_test.py

# 詳細性能測定
python scripts/test_performance.py
```

### 測定指標
- **処理FPS**: 実際の顔検出処理頻度
- **CPU使用率**: システム負荷
- **メモリ使用量**: メモリ消費量
- **検出遅延**: 検出結果の遅延時間

## 🎛️ 推奨設定

### リアルタイム重視
```yaml
max_fps: 15.0
downsample_factor: 0.8
```

### バランス重視
```yaml
max_fps: 10.0
downsample_factor: 0.7
```

### 省電力重視
```yaml
max_fps: 3.0
downsample_factor: 0.5
```

## 💡 実装詳細

### アーキテクチャ変更
1. **フレームキャッシュ**: 最新フレームのみ保持
2. **タイマー処理**: 定期的な処理実行
3. **画像縮小**: 処理前の画像縮小

### 処理フロー
```
カメラ入力 → フレームキャッシュ → タイマー処理 → 画像縮小 → 顔検出
```

## 🔍 トラブルシューティング

### よくある問題

1. **検出精度低下**
   - `downsample_factor` を大きくする（1.0に近づける）
   - `max_fps` を上げる

2. **CPU負荷依然高い**
   - `max_fps` を下げる
   - `downsample_factor` を小さくする

3. **遅延が大きい**
   - `max_fps` を上げる

### 設定例
```bash
# 問題解決例
ros2 run susumu_face_engagement_detector face_detection_node \
  --ros-args \
  -p detection_model:=hog \
  -p max_fps:=8.0 \
  -p downsample_factor:=0.6
```

## 📊 期待される効果

- **CPU負荷**: 50-80%削減
- **消費電力**: 30-60%削減
- **処理遅延**: 設定に応じて調整可能
- **検出精度**: 軽微な低下（実用レベル維持）

---

**注意**: 設定はハードウェア性能と使用目的に応じて調整してください。