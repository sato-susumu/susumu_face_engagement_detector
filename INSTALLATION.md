# インストールガイド

## 依存関係のインストール

### Python依存関係

```bash
pip install face_recognition opencv-python numpy
```

### ROS2依存関係

```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

## ビルド方法

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_face_engagement_detector
source install/setup.bash
```

## 動作確認

```bash
# ビルドが成功したかの確認
ros2 pkg list | grep susumu_face_engagement_detector

# 実行ファイルの確認
ros2 pkg executables susumu_face_engagement_detector
```

## 既知の顔画像設定

```bash
mkdir known_faces
# 既知の人物の顔画像を known_faces/ ディレクトリに配置
# ファイル名がface_idとして使用されます
```

## トラブルシューティング

### 依存関係エラー

**face_recognitionエラー**:
```bash
# cmakeとdlibが必要
sudo apt install cmake libdlib-dev
pip install dlib
pip install face_recognition
```

**OpenCVエラー**:
```bash
pip install opencv-python
# またはシステムパッケージ
sudo apt install python3-opencv
```

### ビルドエラー

**setup.pyエラー**:
```bash
# Python依存関係を再インストール
pip install --upgrade setuptools wheel
```

**ROS2パッケージエラー**:
```bash
# rosdepで依存関係を更新
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```