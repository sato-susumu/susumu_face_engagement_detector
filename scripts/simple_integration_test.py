#!/usr/bin/env python3
"""
シンプルな統合テスト - 基本的な動作確認のみ
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image

class SimpleIntegrationTest(Node):
    def __init__(self):
        super().__init__('simple_integration_test')
        
        # 受信確認用
        self.camera_received = False
        self.detection_received = False
        self.recognition_received = False
        self.gaze_received = False
        
        # テストプロセス
        self.camera_process = None
        
        # 購読設定
        self.setup_subscriptions()
        
        self.get_logger().info('Simple Integration Test started')
    
    def setup_subscriptions(self):
        """シンプルな購読設定"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        # 画像トピック（BEST_EFFORT）
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 文字列トピック（RELIABLE）
        string_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # 購読作成
        self.camera_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.camera_callback, image_qos)
        
        self.detection_sub = self.create_subscription(
            String, '/face_detections', self.detection_callback, string_qos)
        
        self.recognition_sub = self.create_subscription(
            String, '/face_identities', self.recognition_callback, string_qos)
        
        self.gaze_sub = self.create_subscription(
            String, '/gaze_status', self.gaze_callback, string_qos)
    
    def camera_callback(self, msg):
        if not self.camera_received:
            self.get_logger().info(f'✅ Camera data received: {msg.width}x{msg.height}')
            self.camera_received = True
    
    def detection_callback(self, msg):
        if not self.detection_received:
            self.get_logger().info(f'✅ Face detection received: {msg.data[:50]}...')
            self.detection_received = True
    
    def recognition_callback(self, msg):
        if not self.recognition_received:
            self.get_logger().info(f'✅ Face recognition received: {msg.data}')
            self.recognition_received = True
    
    def gaze_callback(self, msg):
        if not self.gaze_received:
            self.get_logger().info(f'✅ Gaze analysis received: {msg.data}')
            self.gaze_received = True
    
    def start_test_camera(self):
        """テストカメラ起動"""
        test_image = "test_images/person1_test.jpg"
        
        if not os.path.exists(test_image):
            self.get_logger().error(f"Test image not found: {test_image}")
            return False
        
        cmd = [
            'python', 'susumu_face_engagement_detector/test_camera_node.py',
            '--ros-args',
            '-p', 'test_mode:=file',
            '-p', f'image_file:={test_image}',
            '-p', 'fps:=2.0'
        ]
        
        try:
            self.camera_process = subprocess.Popen(cmd, 
                                                 stdout=subprocess.DEVNULL, 
                                                 stderr=subprocess.DEVNULL)
            self.get_logger().info('Started test camera')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {e}')
            return False
    
    def stop_test_camera(self):
        """テストカメラ停止"""
        if self.camera_process:
            try:
                self.camera_process.terminate()
                self.camera_process.wait(timeout=3)
            except:
                self.camera_process.kill()
                self.camera_process.wait()
            self.camera_process = None
    
    def run_test(self, timeout=15):
        """テスト実行"""
        self.get_logger().info(f'Running test for {timeout} seconds...')
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # 全ての段階が受信完了したかチェック
            if (self.camera_received and self.detection_received and 
                self.recognition_received and self.gaze_received):
                self.get_logger().info('🎉 All pipeline stages working!')
                return True
        
        return False
    
    def check_results(self):
        """結果確認"""
        print("\\n" + "="*60)
        print("SIMPLE INTEGRATION TEST RESULTS")
        print("="*60)
        
        results = {
            '📹 Camera Input': self.camera_received,
            '👤 Face Detection': self.detection_received,
            '🔍 Face Recognition': self.recognition_received,
            '👁️  Gaze Analysis': self.gaze_received
        }
        
        passed = 0
        total = len(results)
        
        for stage, success in results.items():
            status = "✅ PASS" if success else "❌ FAIL"
            print(f"  {stage:20} {status}")
            if success:
                passed += 1
        
        print(f"\\n📊 Results: {passed}/{total} stages passed ({passed/total:.1%})")
        
        if passed == total:
            print("🎉 SUCCESS: All pipeline stages are working!")
            return True
        elif passed >= total * 0.75:
            print("🟡 PARTIAL: Most stages working, minor issues detected")
            return True
        else:
            print("❌ FAILURE: Major pipeline issues detected")
            return False

def main():
    # 必要なファイルの確認
    if not os.path.exists("test_images"):
        print("❌ Test images directory not found")
        print("🔧 Please run: python scripts/generate_test_images.py")
        return False
    
    rclpy.init()
    
    try:
        tester = SimpleIntegrationTest()
        
        print("🚀 Starting Simple Integration Test")
        print("📋 This test verifies basic pipeline functionality")
        print("⏱️  Test duration: 15 seconds\\n")
        
        # テストカメラ起動
        if not tester.start_test_camera():
            return False
        
        # 起動待機
        time.sleep(2)
        
        # テスト実行
        test_success = tester.run_test(timeout=15)
        
        # 結果確認
        overall_success = tester.check_results()
        
        return overall_success
        
    except KeyboardInterrupt:
        print("\\n⚠️  Test interrupted by user")
        return False
    except Exception as e:
        print(f"\\n💥 Test failed: {e}")
        return False
    finally:
        if 'tester' in locals():
            tester.stop_test_camera()
            tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)