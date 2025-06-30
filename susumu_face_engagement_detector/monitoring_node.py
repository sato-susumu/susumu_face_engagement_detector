#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time
import threading
import os
import sys
from collections import defaultdict, deque
from datetime import datetime
import json

class TopicMonitor:
    def __init__(self, topic_name: str, msg_type):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.message_count = 0
        self.last_message_time = None
        self.message_times = deque(maxlen=100)
        self.message_sizes = deque(maxlen=100)
        self.last_message_content = None
        self.hz = 0.0
        self.avg_size = 0.0
        
    def add_message(self, msg):
        current_time = time.time()
        self.message_count += 1
        self.last_message_time = current_time
        self.message_times.append(current_time)
        
        # メッセージサイズ計算
        if hasattr(msg, 'data'):
            size = len(str(msg.data))
            self.last_message_content = str(msg.data)[:100]  # 最初の100文字のみ
        else:
            size = len(str(msg))
            self.last_message_content = str(msg)[:100]
            
        self.message_sizes.append(size)
        
        # Hz計算
        if len(self.message_times) >= 2:
            time_window = self.message_times[-1] - self.message_times[0]
            if time_window > 0:
                self.hz = (len(self.message_times) - 1) / time_window
        
        # 平均サイズ計算
        if self.message_sizes:
            self.avg_size = sum(self.message_sizes) / len(self.message_sizes)

class FaceEngagementMonitor(Node):
    def __init__(self):
        super().__init__('face_engagement_monitor')
        
        # 監視対象トピック定義（image_topicは後で動的に設定）
        self.topics = {
            '/face_detections': (String, 'output'),
            '/face_identities': (String, 'output'),
            '/gaze_status': (String, 'output'),
            '/face_event': (String, 'output'),
            '/gaze_event': (String, 'output')
        }
        
        # モニター初期化
        self.monitors = {}
        self.topic_subscriptions = {}
        
        # パラメータ
        self.declare_parameter('refresh_rate', 1.0)
        self.declare_parameter('show_content', True)
        self.declare_parameter('log_to_file', False)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().double_value
        self.show_content = self.get_parameter('show_content').get_parameter_value().bool_value
        self.log_to_file = self.get_parameter('log_to_file').get_parameter_value().bool_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # 画像トピックを動的に追加
        self.topics[image_topic] = (Image, 'input')
        self.image_topic = image_topic  # 後で参照用に保存
        
        # 各トピックの購読設定
        self.setup_subscriptions()
        
        # 表示用タイマー
        self.display_timer = self.create_timer(self.refresh_rate, self.display_status)
        
        # ログファイル設定
        if self.log_to_file:
            self.log_file = open(f'face_engagement_monitor_{int(time.time())}.log', 'w')
        
        # 監視対象トピックをログ出力
        input_topics = [topic for topic, (_, direction) in self.topics.items() if direction == 'input']
        output_topics = [topic for topic, (_, direction) in self.topics.items() if direction == 'output']
        self.get_logger().info(f'Face Engagement Monitor started')
        self.get_logger().info(f'Input topics: {", ".join(input_topics)}')
        self.get_logger().info(f'Output topics: {", ".join(output_topics)}')
        
    def setup_subscriptions(self):
        for topic_name, (msg_type, direction) in self.topics.items():
            monitor = TopicMonitor(topic_name, msg_type)
            self.monitors[topic_name] = monitor
            
            # コールバック関数作成
            callback = self.create_callback(topic_name)
            
            # 購読作成（QoS設定を調整）
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            
            # 画像トピック用の設定
            if msg_type == Image:
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    durability=DurabilityPolicy.VOLATILE
                )
            else:
                # その他のトピック用の設定
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    durability=DurabilityPolicy.VOLATILE
                )
            
            subscription = self.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos_profile
            )
            self.topic_subscriptions[topic_name] = subscription
            
    def create_callback(self, topic_name: str):
        def callback(msg):
            if topic_name in self.monitors:
                self.monitors[topic_name].add_message(msg)
        return callback
        
    def display_status(self):
        # 画面クリア
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # ヘッダー表示
        print("=" * 80)
        print("Face Engagement Detection System - Node Monitor")
        print(f"Updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 80)
        
        # ノード状態表示
        self.display_node_status()
        
        # トピック状態表示
        self.display_topic_status()
        
        # パフォーマンス統計
        self.display_performance_stats()
        
        # データフロー表示
        self.display_data_flow()
        
        print("=" * 80)
        print("Press Ctrl+C to exit")
        
        if self.log_to_file:
            self.log_status()
    
    def display_node_status(self):
        print("\n📊 Node Status:")
        print("-" * 40)
        
        # 実際のノード存在チェック
        try:
            # ROS2のAPIを使ってノード一覧を取得
            node_names_and_namespaces = self.get_node_names_and_namespaces()
            node_names = [name for name, namespace in node_names_and_namespaces]
            
            actual_nodes = {
                'face_detection': 'face_detection_node' in node_names,
                'face_recognition': 'face_recognition_node' in node_names,
                'gaze_analysis': 'gaze_analysis_node' in node_names,
                'engagement_manager': 'engagement_manager_node' in node_names
            }
        except Exception as e:
            self.get_logger().debug(f'Failed to get node names: {e}')
            actual_nodes = {
                'face_detection': False,
                'face_recognition': False,
                'gaze_analysis': False,
                'engagement_manager': False
            }
        
        # メッセージ受信による状態判定
        message_status = {
            'face_detection': self.monitors['/face_detections'].message_count > 0,
            'face_recognition': self.monitors['/face_identities'].message_count > 0,
            'gaze_analysis': self.monitors['/gaze_status'].message_count > 0,
            'engagement_manager': (self.monitors['/face_event'].message_count > 0 or 
                                 self.monitors['/gaze_event'].message_count > 0)
        }
        
        for node_name in actual_nodes.keys():
            node_exists = actual_nodes[node_name]
            has_messages = message_status[node_name]
            
            if node_exists and has_messages:
                status = "🟢 ACTIVE"
            elif node_exists and not has_messages:
                status = "🟡 RUNNING (no messages)"
            else:
                status = "🔴 INACTIVE"
                
            print(f"  {node_name:20} {status}")
        
        # デバッグ情報：検出されたノード一覧を表示
        try:
            node_names_and_namespaces = self.get_node_names_and_namespaces()
            detected_nodes = [name for name, namespace in node_names_and_namespaces if 'face' in name or 'gaze' in name or 'engagement' in name]
            if detected_nodes:
                print(f"\n  🔍 Detected face/gaze nodes: {', '.join(detected_nodes)}")
        except Exception as e:
            print(f"\n  ⚠️ Node detection error: {str(e)[:50]}...")
    
    def display_topic_status(self):
        print("\n📡 Topic Status:")
        print("-" * 80)
        print(f"{'Topic':25} {'Type':10} {'Count':8} {'Hz':8} {'Size':8} {'Last Msg'}")
        print("-" * 80)
        
        for topic_name, monitor in self.monitors.items():
            msg_type = "Image" if monitor.msg_type == Image else "String"
            count = monitor.message_count
            hz = f"{monitor.hz:.1f}" if monitor.hz > 0 else "0.0"
            size = f"{monitor.avg_size:.0f}B" if monitor.avg_size > 0 else "0B"
            
            # 最終メッセージ時刻
            if monitor.last_message_time:
                elapsed = time.time() - monitor.last_message_time
                if elapsed < 1:
                    last_msg = "< 1s"
                elif elapsed < 60:
                    last_msg = f"{elapsed:.0f}s"
                else:
                    last_msg = f"{elapsed/60:.0f}m"
            else:
                last_msg = "None"
            
            print(f"{topic_name:25} {msg_type:10} {count:8} {hz:8} {size:8} {last_msg}")
    
    def display_performance_stats(self):
        print("\n⚡ Performance Statistics:")
        print("-" * 40)
        
        # 全体の処理レート
        input_hz = self.monitors[self.image_topic].hz
        output_hz = max([
            self.monitors['/face_detections'].hz,
            self.monitors['/face_identities'].hz,
            self.monitors['/gaze_status'].hz
        ])
        
        print(f"  Input Rate:    {input_hz:.1f} Hz")
        print(f"  Output Rate:   {output_hz:.1f} Hz")
        
        if input_hz > 0:
            efficiency = (output_hz / input_hz) * 100
            print(f"  Efficiency:    {efficiency:.1f}%")
        
        # 遅延推定
        if input_hz > 0 and output_hz > 0:
            estimated_latency = 1.0 / min(input_hz, output_hz)
            print(f"  Est. Latency:  {estimated_latency*1000:.0f}ms")
    
    def display_data_flow(self):
        print("\n🔄 Data Flow:")
        print("-" * 40)
        
        # パイプライン表示（画像トピックを動的に設定）
        pipeline = [
            (self.image_topic, "📹"),
            ("/face_detections", "👤"),
            ("/face_identities", "🔍"),
            ("/gaze_status", "👁️"),
            ("/face_event", "📢"),
            ("/gaze_event", "📢")
        ]
        
        for i, (topic, icon) in enumerate(pipeline):
            monitor = self.monitors[topic]
            active = "✅" if monitor.message_count > 0 else "❌"
            
            if i == 0:
                print(f"  {icon} {topic} {active}")
            else:
                print(f"    ↓")
                print(f"  {icon} {topic} {active}")
        
        # メッセージ内容表示（オプション）
        if self.show_content:
            print("\n📝 Latest Messages:")
            print("-" * 40)
            for topic_name, monitor in self.monitors.items():
                if monitor.last_message_content and topic_name != self.image_topic:
                    content = monitor.last_message_content
                    if len(content) > 60:
                        content = content[:60] + "..."
                    print(f"  {topic_name}: {content}")
    
    def log_status(self):
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'topics': {}
        }
        
        for topic_name, monitor in self.monitors.items():
            log_data['topics'][topic_name] = {
                'count': monitor.message_count,
                'hz': monitor.hz,
                'avg_size': monitor.avg_size,
                'last_message_time': monitor.last_message_time
            }
        
        self.log_file.write(json.dumps(log_data) + '\n')
        self.log_file.flush()
    
    def destroy_node(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = FaceEngagementMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nShutting down monitor...")
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()