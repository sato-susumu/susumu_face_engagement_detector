# 頻用コマンドパターン

## 開発・テスト

### ビルド・テスト
```bash
# パッケージビルド
colcon build --packages-select susumu_face_engagement_detector

# テスト実行
pytest                              # 全テスト
pytest -v                         # 詳細出力
pytest -m "not integration"       # 単体テストのみ
pytest -m integration             # 統合テストのみ
pytest test/test_specific.py      # 特定ファイル

# カバレッジ付きテスト
pytest --cov=susumu_face_engagement_detector --cov-report=html

# 並列テスト実行
pytest -n auto
```

### ソース更新
```bash
# 環境設定
source /opt/ros/humble/setup.bash
source install/setup.bash

# パッケージ再ビルド（開発時）
colcon build --packages-select susumu_face_engagement_detector --symlink-install
```

## ROS2操作

### ノード起動
```bash
# 個別ノード起動
ros2 run susumu_face_engagement_detector face_detection_node
ros2 run susumu_face_engagement_detector face_recognition_node
ros2 run susumu_face_engagement_detector gaze_analysis_node
ros2 run susumu_face_engagement_detector engagement_manager_node

# ランチファイル使用
ros2 launch susumu_face_engagement_detector simple_launch.py
ros2 launch susumu_face_engagement_detector multi_node_launch.py
ros2 launch susumu_face_engagement_detector face_detection_only.launch.py

# 元の統合ノード
ros2 run susumu_face_engagement_detector face_engagement_node
```

### 監視・デバッグ
```bash
# 統合モニタリング（推奨）
ros2 launch susumu_face_engagement_detector monitoring.launch.py

# 個別モニタリング
ros2 run susumu_face_engagement_detector monitoring_node

# カスタムモニタリング
ros2 launch susumu_face_engagement_detector monitoring.launch.py refresh_rate:=0.5 show_content:=false

# 従来のコマンド
ros2 node list
ros2 topic list

# トピック監視
ros2 topic echo /face_detections
ros2 topic echo /face_identities
ros2 topic echo /gaze_status
ros2 topic echo /face_event
ros2 topic echo /gaze_event

# トピック周波数確認
ros2 topic hz /image
ros2 topic hz /face_detections

# ノード情報確認
ros2 node info /face_detection_node
ros2 node info /face_recognition_node
ros2 node info /gaze_analysis_node
ros2 node info /engagement_manager_node

# パラメータ確認
ros2 param list /face_detection_node
ros2 param get /face_detection_node detection_model
ros2 param set /face_detection_node detection_model cnn
```

### サービス・アクション
```bash
# サービス一覧
ros2 service list

# サービス呼び出し
ros2 service call /service_name service_type "request_data"
```

## システム監視

### リソース監視
```bash
# CPU・メモリ使用量
htop
top -p $(pgrep -f "face_")

# ROS2プロセス確認
ps aux | grep ros2
ps aux | grep face_

# GPU使用量（該当する場合）
nvidia-smi
```

### ログ確認
```bash
# ROS2ログ
ros2 log list
ros2 log level /face_detection_node DEBUG

# システムログ
journalctl -f | grep face_
tail -f ~/.ros/log/latest/face_detection_node/stdout.log
```

## トラブルシューティング

### 依存関係問題
```bash
# 依存関係確認
rosdep check --from-paths src --ignore-src -r -y

# 依存関係インストール
rosdep install --from-paths src --ignore-src -r -y

# Python依存関係
pip install face_recognition opencv-python numpy
pip list | grep face_recognition
```

### 通信問題
```bash
# ネットワーク確認
ros2 doctor
ros2 daemon stop && ros2 daemon start

# 環境変数確認
echo $ROS_DOMAIN_ID
echo $ROS_DISTRO
env | grep ROS
```

### パフォーマンス問題
```bash
# CPU使用率
ros2 topic hz /image --window 100

# メモリ使用量
ros2 node info /face_detection_node | grep -A5 "Memory"

# 計算時間測定
time ros2 topic echo /face_detections --once
```

## 設定・カスタマイズ

### パラメータ設定
```bash
# 設定確認
ros2 param get /face_detection_node detection_model
ros2 param get /face_recognition_node match_tolerance
ros2 param get /gaze_analysis_node gaze_threshold_px

# 設定変更
ros2 param set /face_detection_node detection_model cnn
ros2 param set /face_recognition_node match_tolerance 0.5
ros2 param set /gaze_analysis_node gaze_threshold_px 30

# 設定保存
ros2 param dump /face_detection_node --output-dir config/
```

### カスタマイズ起動
```bash
# カスタムパラメータで起動
ros2 run susumu_face_engagement_detector face_detection_node --ros-args -p detection_model:=cnn

# パラメータファイル指定
ros2 launch susumu_face_engagement_detector simple_launch.py params_file:=config/custom_params.yaml

# 名前空間指定
ros2 run susumu_face_engagement_detector face_detection_node --ros-args -r __ns:=/robot1
```

## 開発効率化

### よく使うエイリアス
```bash
# .bashrcに追加推奨
alias build_face='colcon build --packages-select susumu_face_engagement_detector'
alias test_face='pytest -v'
alias run_face='ros2 launch susumu_face_engagement_detector simple_launch.py'
alias monitor_face='ros2 launch susumu_face_engagement_detector monitoring.launch.py'
alias monitor_simple='ros2 run susumu_face_engagement_detector monitoring_node'
```

### 環境切り替え
```bash
# 開発環境
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash

# テスト環境
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## データ収集・分析

### ログ収集
```bash
# 全ログ収集
ros2 bag record -a -o face_engagement_data

# 特定トピックのみ
ros2 bag record /image /face_detections /face_identities -o face_detection_data

# 再生
ros2 bag play face_engagement_data
```

### パフォーマンス分析
```bash
# 処理時間測定
ros2 run demo_nodes_cpp listener --ros-args --log-level DEBUG

# 統計情報取得
ros2 topic bw /face_detections    # 帯域幅
ros2 topic delay /face_detections # 遅延時間
```

## Git操作

### 一般的なワークフロー
```bash
# 現在の状態確認
git status
git log --oneline -10

# 変更のコミット
git add .
git commit -m "feat: add new face detection feature"

# プッシュ
git push origin main

# ブランチ操作
git checkout -b feature/new-detection
git merge main
```

### 緊急時対応
```bash
# 変更の取り消し
git checkout -- filename
git reset HEAD filename

# 強制リセット（注意）
git reset --hard HEAD~1
```