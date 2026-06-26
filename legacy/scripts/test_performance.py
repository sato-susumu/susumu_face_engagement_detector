#!/usr/bin/env python3
"""
Face Detection Performance Test
CPU負荷軽減機能のテスト
"""

import rclpy
import time
import subprocess
import os
import signal
from rclpy.node import Node
from std_msgs.msg import String
from collections import defaultdict


class PerformanceTestNode(Node):
    def __init__(self):
        super().__init__('performance_test_node')
        self.message_count = 0
        self.start_time = time.time()
        self.message_times = []
        
        # 顔検出トピックを購読
        self.create_subscription(String, '/face_detections', self.on_face_detection, 10)
        
        self.get_logger().info('Performance test started')
    
    def on_face_detection(self, msg):
        """顔検出メッセージを受信"""
        current_time = time.time()
        self.message_count += 1
        self.message_times.append(current_time)
        
        # 30秒間の統計を取得
        if current_time - self.start_time >= 30:
            self.print_statistics()
            self.start_time = current_time
            self.message_count = 0
            self.message_times = []
    
    def print_statistics(self):
        """統計情報を表示"""
        elapsed = time.time() - self.start_time
        fps = self.message_count / elapsed if elapsed > 0 else 0
        
        print(f"\\n=== Performance Statistics (30s) ===")
        print(f"Total messages: {self.message_count}")
        print(f"Average FPS: {fps:.2f}")
        print(f"Elapsed time: {elapsed:.2f}s")
        
        # 処理間隔の統計
        if len(self.message_times) >= 2:
            intervals = [self.message_times[i] - self.message_times[i-1] 
                        for i in range(1, len(self.message_times))]
            avg_interval = sum(intervals) / len(intervals)
            print(f"Average interval: {avg_interval:.3f}s")
        
        print("="*40)


def run_performance_test(config_name, config_params):
    """性能テストを実行"""
    print(f"\\n🚀 Testing configuration: {config_name}")
    print(f"Parameters: {config_params}")
    
    # パラメータファイルを作成
    param_file = f"/tmp/test_params_{config_name}.yaml"
    with open(param_file, 'w') as f:
        f.write("face_detection_node:\\n")
        f.write("  ros__parameters:\\n")
        f.write("    image_topic: '/camera/color/image_raw'\\n")
        f.write("    detection_model: 'hog'\\n")
        for key, value in config_params.items():
            f.write(f"    {key}: {value}\\n")
    
    # テストカメラを起動
    camera_cmd = [
        'python', 'susumu_face_engagement_detector/test_camera_node.py',
        '--ros-args',
        '-p', 'test_mode:=synthetic',
        '-p', 'fps:=30.0'  # 高FPSで入力
    ]
    
    # 顔検出ノードを起動
    detection_cmd = [
        'python', 'susumu_face_engagement_detector/face_detection_node.py',
        '--ros-args',
        '--params-file', param_file
    ]
    
    camera_proc = None
    detection_proc = None
    
    try:
        # プロセス起動
        camera_proc = subprocess.Popen(camera_cmd)
        time.sleep(2)  # カメラ起動待機
        
        detection_proc = subprocess.Popen(detection_cmd)
        time.sleep(3)  # 顔検出ノード起動待機
        
        # 60秒間テスト実行
        print(f"Running {config_name} test for 60 seconds...")
        
        rclpy.init()
        test_node = PerformanceTestNode()
        
        start_time = time.time()
        while time.time() - start_time < 60:
            rclpy.spin_once(test_node, timeout_sec=1.0)
        
        test_node.destroy_node()
        rclpy.shutdown()
        
        print(f"✅ {config_name} test completed")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
    finally:
        # プロセス終了
        if camera_proc:
            camera_proc.terminate()
            camera_proc.wait()
        if detection_proc:
            detection_proc.terminate()
            detection_proc.wait()
        
        # 一時ファイル削除
        if os.path.exists(param_file):
            os.remove(param_file)


def main():
    """メイン関数"""
    # 異なる設定でテスト
    test_configs = {
        'high_performance': {
            'max_fps': 0.0,
            'downsample_factor': 1.0
        },
        'balanced': {
            'max_fps': 10.0,
            'downsample_factor': 0.7
        },
        'low_power': {
            'max_fps': 5.0,
            'downsample_factor': 0.5
        }
    }
    
    print("🎯 Face Detection Performance Test")
    print("Testing CPU load reduction features...")
    
    for config_name, config_params in test_configs.items():
        run_performance_test(config_name, config_params)
        time.sleep(5)  # テスト間の待機
    
    print("\\n🎉 All performance tests completed!")


if __name__ == '__main__':
    main()