# API仕様

## トピック構成

```
/camera/color/image_raw (sensor_msgs/Image) → face_detection_node
  ↓ face_detections (std_msgs/String)
face_recognition_node
  ↓ face_identities (std_msgs/String)
gaze_analysis_node
  ↓ gaze_status (std_msgs/String)
engagement_manager_node
  ↓ face_event, gaze_event (std_msgs/String)
```

## メッセージフォーマット

### 内部メッセージ

#### face_detections
```
face_idx|center_x|center_y|width|height|frame_width|frame_height|feature_csv
```
例: `0|320|240|100|120|640|480|-0.123,0.456,0.789,...`

#### face_identities
```
face_id|center_x|center_y|width|height|frame_width|frame_height
```
例: `user_1|320|240|100|120|640|480`

#### gaze_status
```
face_id|status|center_x|center_y|width|height
```
- status: `TRACKING`, `ENGAGED`, `DISENGAGED`
- 例: `user_1|ENGAGED|320|240|100|120`

### 出力イベントメッセージ

#### face_event
- `{face_id}:DETECTED` - 顔検出
- `{face_id}:LOST` - 顔喪失

例:
- `user_1:DETECTED`
- `user_1:LOST`

#### gaze_event
- `{face_id}:ENGAGED` - 注視開始
- `{face_id}:DISENGAGED` - 注視終了

例:
- `user_1:ENGAGED`
- `user_1:DISENGAGED`

## ノード詳細

### face_detection_node
**機能**: 画像から顔を検出し、顔特徴量を生成

**入力**:
- `/camera/color/image_raw` (sensor_msgs/Image)

**出力**:
- `/face_detections` (std_msgs/String)

**パラメータ**:
- `image_topic`: 入力画像トピック名
- `detection_model`: 検出モデル (`hog` | `cnn`)

### face_recognition_node
**機能**: 顔特徴量から人物を識別

**入力**:
- `/face_detections` (std_msgs/String)

**出力**:
- `/face_identities` (std_msgs/String)

**パラメータ**:
- `known_faces_dir`: 既知顔画像ディレクトリ
- `match_tolerance`: マッチング閾値 (0.0-1.0)

### gaze_analysis_node
**機能**: 顔の位置から注視状態を判定

**入力**:
- `/face_identities` (std_msgs/String)

**出力**:
- `/gaze_status` (std_msgs/String)

**パラメータ**:
- `gaze_threshold_px`: 注視判定閾値（ピクセル）
- `gaze_duration`: 注視判定持続時間（秒）

### engagement_manager_node
**機能**: 全体の状態管理とイベント発行

**入力**:
- `/gaze_status` (std_msgs/String)

**出力**:
- `/face_event` (std_msgs/String)
- `/gaze_event` (std_msgs/String)

**パラメータ**:
- `face_timeout`: 顔喪失タイムアウト（秒）

## 動作仕様

### 顔検出・認識の流れ

1. **顔検出**: カメラ画像から顔を検出し、顔特徴量を生成
2. **顔認識**: 
   - 既知の顔画像と一致すれば優先的に識別
   - 過去に検出した顔と一致すれば追跡
   - 新規の場合は新しいIDを割り当て（`user_1`, `user_2`, ...）

### 注視判定の仕組み

1. **位置判定**: 顔の中心がカメラ中央付近（閾値内）にあるかチェック
2. **持続判定**: 指定時間継続して中央を向いているかチェック
3. **状態更新**: `TRACKING` → `ENGAGED` → `DISENGAGED` の状態遷移

### イベント発行タイミング

#### face_event
- `DETECTED`: 新しい顔が検出された時
- `LOST`: 顔が一定時間検出されなくなった時

#### gaze_event
- `ENGAGED`: 注視状態が開始された時
- `DISENGAGED`: 注視状態が終了された時

## QoS設定

### 画像トピック
- **Reliability**: BEST_EFFORT
- **History**: KEEP_LAST (depth=1)
- **Durability**: VOLATILE

### 文字列トピック
- **Reliability**: RELIABLE
- **History**: KEEP_LAST (depth=10)
- **Durability**: VOLATILE