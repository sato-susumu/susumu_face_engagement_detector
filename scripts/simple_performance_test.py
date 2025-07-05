#!/usr/bin/env python3
"""
Simple Performance Test
CPU負荷軽減機能の基本動作確認
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SimplePerformanceTest(Node):
    def __init__(self):
        super().__init__('simple_performance_test')
        self.message_count = 0
        self.start_time = time.time()
        
        # 顔検出トピックを購読
        self.create_subscription(String, '/face_detections', self.on_face_detection, 10)
        
        # 定期的に統計表示
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Simple performance test started')
    
    def on_face_detection(self, msg):
        """顔検出メッセージを受信"""
        self.message_count += 1
        
        # メッセージ内容を確認
        if self.message_count <= 3:
            self.get_logger().info(f'Face detection message {self.message_count}: {msg.data[:50]}...')
    
    def print_stats(self):
        """統計情報を表示"""
        elapsed = time.time() - self.start_time
        fps = self.message_count / elapsed if elapsed > 0 else 0
        
        print(f"\\n--- Performance Stats ---")
        print(f"Messages received: {self.message_count}")
        print(f"Elapsed time: {elapsed:.1f}s")
        print(f"Average FPS: {fps:.2f}")
        print("------------------------")


def main():
    rclpy.init()
    
    try:
        test_node = SimplePerformanceTest()
        print("\\n🎯 Simple performance test running...")
        print("💡 Start face detection node with different performance settings to test")
        print("📝 Example: ros2 run susumu_face_engagement_detector face_detection_node --ros-args -p max_fps:=3.0 -p downsample_factor:=0.5")
        
        rclpy.spin(test_node)
        
    except KeyboardInterrupt:
        print("\\n⚠️  Test stopped by user")
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()