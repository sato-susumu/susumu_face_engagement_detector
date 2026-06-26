# 顔エンゲージメント検出システム - テスト手順

## 概要

このドキュメントでは、顔エンゲージメント検出システムの動作検証方法について説明します。疑似的なカメラデータを使用してシステム全体の動作を確認できます。

## テスト環境構築

### 1. 合成テスト画像の生成

著作権フリーの合成顔画像を生成します：

```bash
# パッケージディレクトリに移動
cd /home/taro/ros2_ws/src/susumu_face_engagement_detector

# テスト画像生成
python scripts/generate_test_images.py
```

生成される画像：
- `test_images/person1_test.jpg` - 人物1の顔画像
- `test_images/person2_test.jpg` - 人物2の顔画像  
- `test_images/person3_test.jpg` - 人物3の顔画像
- `test_images/multiple_faces_test.jpg` - 複数人の顔画像
- `test_images/no_face_test.jpg` - 顔なし画像
- 各種解像度の画像

### 2. パッケージビルド

テスト用ノードを含めてパッケージをビルドします：

```bash
cd /home/taro/ros2_ws
colcon build --packages-select susumu_face_engagement_detector
source install/setup.bash
```

## テスト実行方法

### 方法1: 合成カメラのみテスト

動的に生成される合成顔画像をテストします：

```bash
# 合成カメラのみ起動（動的顔生成）
python susumu_face_engagement_detector/test_camera_node.py --ros-args -p fps:=5.0

# または静止画像を使用
python susumu_face_engagement_detector/test_camera_node.py --ros-args \
  -p test_mode:=file \
  -p image_file:=test_images/person1_test.jpg \
  -p fps:=3.0
```

### 方法2: パイプライン全体テスト

#### 2.1 別々のターミナルで実行

**ターミナル1: 合成カメラ起動**
```bash
cd /home/taro/ros2_ws/src/susumu_face_engagement_detector
source ../../install/setup.bash
python susumu_face_engagement_detector/test_camera_node.py --ros-args -p fps:=3.0
```

**ターミナル2: 顔検出システム起動**
```bash
cd /home/taro/ros2_ws/src/susumu_face_engagement_detector
source ../../install/setup.bash
ros2 launch susumu_face_engagement_detector simple_launch.py
```

**ターミナル3: 監視ツール起動**
```bash
cd /home/taro/ros2_ws/src/susumu_face_engagement_detector
source ../../install/setup.bash
ros2 run susumu_face_engagement_detector monitoring_node
```

**ターミナル4: パイプライン検証**
```bash
cd /home/taro/ros2_ws/src/susumu_face_engagement_detector
source ../../install/setup.bash
python scripts/test_pipeline.py
```

#### 2.2 統合ランチファイル（準備中）

```bash
# 合成カメラ + パイプライン全体
ros2 launch susumu_face_engagement_detector test_with_synthetic_camera.launch.py fps:=5.0

# カメラのみ
ros2 launch susumu_face_engagement_detector test_camera_only.launch.py fps:=10.0 test_mode:=file image_file:=test_images/multiple_faces_test.jpg
```

## テスト項目と期待結果

### 1. 合成カメラテスト

**期待される動作:**
- `/camera/color/image_raw` トピックに画像が配信される
- 合成顔がアニメーションで移動する
- フレーム番号が表示される

**確認コマンド:**
```bash
# 画像トピック確認
ros2 topic echo /camera/color/image_raw --once

# フレームレート確認
ros2 topic hz /camera/color/image_raw
```

### 2. 顔検出テスト

**期待される動作:**
- `/face_detections` トピックにメッセージが配信される
- メッセージフォーマット: `face_idx|center_x|center_y|width|height|frame_width|frame_height|feature_csv`

**確認コマンド:**
```bash
ros2 topic echo /face_detections
```

### 3. 顔認識テスト

**期待される動作:**
- `/face_identities` トピックにメッセージが配信される
- メッセージフォーマット: `face_id|center_x|center_y|width|height|frame_width|frame_height`
- 新しい顔には `user_1`, `user_2` などのIDが付与される

**確認コマンド:**
```bash
ros2 topic echo /face_identities
```

### 4. 注視分析テスト

**期待される動作:**
- `/gaze_status` トピックにメッセージが配信される
- メッセージフォーマット: `face_id|status|center_x|center_y|width|height`
- `status` は `ENGAGED` または `DISENGAGED`

**確認コマンド:**
```bash
ros2 topic echo /gaze_status
```

### 5. イベント生成テスト

**期待される動作:**
- `/face_event` トピック: `face_id:DETECTED` または `face_id:LOST`
- `/gaze_event` トピック: `face_id:ENGAGED` または `face_id:DISENGAGED`

**確認コマンド:**
```bash
ros2 topic echo /face_event
ros2 topic echo /gaze_event
```

## パフォーマンス測定

### 監視ツールでの確認

監視ツールで以下の項目を確認：
- 各ノードの状態（🟢 ACTIVE / 🟡 RUNNING / 🔴 INACTIVE）
- メッセージ頻度（Hz）
- パイプライン効率（入力vs出力）
- データフロー状況

### パイプライン検証スクリプト

30秒間の自動検証：
```bash
python scripts/test_pipeline.py
```

**期待される結果:**
- Camera Input: ✅ PASS
- Face Detection: ✅ PASS
- Face Recognition: ✅ PASS
- Gaze Analysis: ✅ PASS
- Event Generation: ✅ PASS
- Success Rate: 100%

## トラブルシューティング

### よくある問題

1. **カメラ画像が配信されない**
   - `test_camera_node.py` が正常に起動しているか確認
   - トピック名が一致しているか確認（デフォルト: `/camera/color/image_raw`）

2. **顔検出が動作しない**
   - `face_recognition` パッケージがインストールされているか確認
   - CPU使用率が高い場合はFPSを下げる

3. **メッセージフォーマットエラー**
   - パイプライン検証スクリプトでフォーマット妥当性を確認
   - 各ノードのログを確認

4. **QoS互換性問題**
   - Image トピック: BEST_EFFORT
   - String トピック: RELIABLE
   - 監視ツールが正しいQoSプロファイルを使用しているか確認

### デバッグコマンド

```bash
# ノード一覧確認
ros2 node list

# トピック一覧確認
ros2 topic list

# トピック詳細確認
ros2 topic info /camera/color/image_raw

# ノードの依存関係確認
ros2 node info /face_detection_node
```

## テスト画像について

生成されるテスト画像は以下の特徴があります：
- **完全に合成された画像**: 実在の人物は含まれません
- **著作権フリー**: 商用・非商用問わず使用可能
- **多様な設定**: 単一顔、複数顔、顔なし、各種解像度
- **アニメーション対応**: 合成カメラでは動的な顔移動をサポート

これらの画像は顔検出・認識アルゴリズムの基本的な動作確認に適していますが、実際の運用環境とは異なる可能性があります。

## 今後の拡張

1. **より現実的なテストデータ**
   - 多様な角度・照明条件
   - 部分的な遮蔽状況
   - 複数人の複雑な配置

2. **自動化テスト**
   - CI/CDパイプラインでの自動検証
   - 回帰テスト用のベンチマーク

3. **パフォーマンステスト**
   - 高解像度画像での処理能力測定
   - リアルタイム性能の検証