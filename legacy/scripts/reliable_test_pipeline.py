#!/usr/bin/env python3
"""
信頼性の高いパイプライン検証スクリプト
事前に準備されたテスト画像を使用して確実に動作検証を行う
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import signal
import os
import sys
from collections import defaultdict
from std_msgs.msg import String
from sensor_msgs.msg import Image

class ReliablePipelineValidator(Node):
    def __init__(self):
        super().__init__('reliable_pipeline_validator')
        
        # 検証対象トピック
        self.topics = {
            '/camera/color/image_raw': Image,
            '/face_detections': String,
            '/face_identities': String,
            '/gaze_status': String,
            '/face_event': String,
            '/gaze_event': String
        }
        
        # メッセージ収集用
        self.messages = defaultdict(list)
        self.message_times = defaultdict(list)
        self.subscribers = {}
        
        # テスト状態
        self.test_start_time = None
        self.camera_process = None
        self.validation_results = {}
        
        # 各トピックの購読設定
        self.setup_subscriptions()
        
        self.get_logger().info('Reliable Pipeline Validator started')
    
    def setup_subscriptions(self):
        """各トピックの購読設定"""
        for topic_name, msg_type in self.topics.items():
            callback = self.create_callback(topic_name)
            
            # QoS設定
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            if msg_type == Image:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
            else:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
            
            subscriber = self.create_subscription(msg_type, topic_name, callback, qos)
            self.subscribers[topic_name] = subscriber
    
    def create_callback(self, topic_name):
        """コールバック関数生成"""
        def callback(msg):
            current_time = time.time()
            
            # メッセージ内容を保存
            if hasattr(msg, 'data'):
                content = msg.data
            else:
                content = f"Image({msg.width}x{msg.height})"
            
            self.messages[topic_name].append(content)
            self.message_times[topic_name].append(current_time)
            
            # 最新の20メッセージのみ保持
            if len(self.messages[topic_name]) > 20:
                self.messages[topic_name].pop(0)
                self.message_times[topic_name].pop(0)
                
        return callback
    
    def start_test_camera(self, image_file="test_images/person1_test.jpg", fps=3.0):
        """テスト用カメラを起動"""
        cmd = [
            'python', 'susumu_face_engagement_detector/test_camera_node.py',
            '--ros-args',
            '-p', f'test_mode:=file',
            '-p', f'image_file:={image_file}',
            '-p', f'fps:={fps}'
        ]
        
        try:
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.get_logger().info(f'Started test camera with {image_file} at {fps} FPS')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start test camera: {e}')
            return False
    
    def stop_test_camera(self):
        """テスト用カメラを停止"""
        if self.camera_process:
            try:
                self.camera_process.terminate()
                self.camera_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.camera_process.kill()
                self.camera_process.wait()
            except Exception as e:
                self.get_logger().error(f'Error stopping camera: {e}')
            finally:
                self.camera_process = None
    
    def run_validation_test(self, duration=20):
        """検証テストを実行"""
        self.get_logger().info(f'Running validation test for {duration} seconds...')
        self.test_start_time = time.time()
        
        # メッセージ収集をリセット
        self.messages.clear()
        self.message_times.clear()
        
        # 指定時間待機
        end_time = time.time() + duration
        last_report_time = time.time()
        
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # 5秒ごとに進捗報告
            if time.time() - last_report_time >= 5:
                elapsed = time.time() - self.test_start_time
                self.print_progress_report(elapsed)
                last_report_time = time.time()
        
        # 結果分析
        return self.analyze_results()
    
    def print_progress_report(self, elapsed_time):
        """進捗報告を表示"""
        print(f"\\n--- Progress Report ({elapsed_time:.1f}s elapsed) ---")
        for topic_name in self.topics.keys():
            count = len(self.messages[topic_name])
            if count > 0:
                latest = self.messages[topic_name][-1]
                if topic_name != '/camera/color/image_raw':
                    print(f"  {topic_name}: {count} msgs - {latest[:60]}...")
                else:
                    print(f"  {topic_name}: {count} msgs - {latest}")
            else:
                print(f"  {topic_name}: 0 msgs")
    
    def analyze_results(self):
        """結果を分析"""
        results = {
            'timestamp': time.time(),
            'duration': time.time() - self.test_start_time if self.test_start_time else 0,
            'topics': {},
            'pipeline_stages': {},
            'overall_status': 'UNKNOWN'
        }
        
        # 各トピックの分析
        for topic_name in self.topics.keys():
            msg_count = len(self.messages[topic_name])
            
            if msg_count > 0:
                # Hz計算
                if len(self.message_times[topic_name]) >= 2:
                    time_span = self.message_times[topic_name][-1] - self.message_times[topic_name][0]
                    hz = (len(self.message_times[topic_name]) - 1) / time_span if time_span > 0 else 0
                else:
                    hz = 0
                
                results['topics'][topic_name] = {
                    'message_count': msg_count,
                    'hz': hz,
                    'status': 'ACTIVE',
                    'latest_message': self.messages[topic_name][-1] if self.messages[topic_name] else None
                }
            else:
                results['topics'][topic_name] = {
                    'message_count': 0,
                    'hz': 0,
                    'status': 'INACTIVE',
                    'latest_message': None
                }
        
        # パイプライン段階の評価
        camera_active = results['topics']['/camera/color/image_raw']['status'] == 'ACTIVE'
        detection_active = results['topics']['/face_detections']['status'] == 'ACTIVE'
        recognition_active = results['topics']['/face_identities']['status'] == 'ACTIVE'
        gaze_active = results['topics']['/gaze_status']['status'] == 'ACTIVE'
        events_active = (results['topics']['/face_event']['status'] == 'ACTIVE' or
                        results['topics']['/gaze_event']['status'] == 'ACTIVE')
        
        results['pipeline_stages'] = {
            'camera_input': camera_active,
            'face_detection': detection_active,
            'face_recognition': recognition_active,
            'gaze_analysis': gaze_active,
            'event_generation': events_active
        }
        
        # 総合評価
        active_stages = sum(results['pipeline_stages'].values())
        total_stages = len(results['pipeline_stages'])
        
        if active_stages == total_stages:
            results['overall_status'] = 'PASS'
        elif active_stages >= total_stages * 0.8:
            results['overall_status'] = 'PARTIAL'
        else:
            results['overall_status'] = 'FAIL'
        
        results['success_rate'] = active_stages / total_stages
        
        return results
    
    def print_detailed_results(self, results):
        """詳細結果を表示"""
        print(f"\\n{'='*80}")
        print("RELIABLE PIPELINE VALIDATION RESULTS")
        print(f"{'='*80}")
        
        print(f"\\n🕒 Test Duration: {results['duration']:.1f} seconds")
        print(f"🎯 Overall Status: {results['overall_status']}")
        print(f"📊 Success Rate: {results['success_rate']:.1%}")
        
        # パイプライン段階結果
        print(f"\\n🔄 Pipeline Stages:")
        print("-" * 50)
        stage_names = {
            'camera_input': '📹 Camera Input',
            'face_detection': '👤 Face Detection',
            'face_recognition': '🔍 Face Recognition',
            'gaze_analysis': '👁️  Gaze Analysis',
            'event_generation': '📢 Event Generation'
        }
        
        for stage_key, stage_name in stage_names.items():
            status = "✅ PASS" if results['pipeline_stages'][stage_key] else "❌ FAIL"
            print(f"  {stage_name:20} {status}")
        
        # トピック詳細
        print(f"\\n📡 Topic Details:")
        print("-" * 80)
        print(f"{'Topic':25} {'Status':10} {'Count':8} {'Hz':8} {'Latest Message'}")
        print("-" * 80)
        
        for topic_name, topic_data in results['topics'].items():
            status = "✅ ACTIVE" if topic_data['status'] == 'ACTIVE' else "❌ INACTIVE"
            count = topic_data['message_count']
            hz = f"{topic_data['hz']:.1f}" if topic_data['hz'] > 0 else "0.0"
            
            latest = topic_data['latest_message']
            if latest and topic_name != '/camera/color/image_raw':
                latest_display = latest[:40] + "..." if len(latest) > 40 else latest
            elif latest:
                latest_display = latest
            else:
                latest_display = "None"
            
            print(f"{topic_name:25} {status:10} {count:8} {hz:8} {latest_display}")
        
        # メッセージフォーマット検証
        self.validate_message_formats(results)
        
        # 推奨事項
        print(f"\\n💡 Recommendations:")
        if results['overall_status'] == 'PASS':
            print("  🎉 All pipeline stages are working correctly!")
            print("  ✅ System is ready for production use.")
        elif results['overall_status'] == 'PARTIAL':
            print("  🟡 Most pipeline stages are working.")
            inactive_stages = [name for name, active in results['pipeline_stages'].items() if not active]
            print(f"  ⚠️  Check these stages: {', '.join(inactive_stages)}")
        else:
            print("  🔴 Multiple pipeline stages are failing.")
            print("  🔧 Check ROS2 node startup and dependencies.")
            print("  📋 Review logs for error messages.")
    
    def validate_message_formats(self, results):
        """メッセージフォーマットを検証"""
        print(f"\\n📝 Message Format Validation:")
        print("-" * 50)
        
        format_results = {}
        
        # 顔検出メッセージ検証
        if '/face_detections' in results['topics'] and results['topics']['/face_detections']['latest_message']:
            try:
                msg = results['topics']['/face_detections']['latest_message']
                parts = msg.split('|')
                if len(parts) == 8:
                    format_results['face_detections'] = "✅ Valid format"
                else:
                    format_results['face_detections'] = f"❌ Invalid format (expected 8 parts, got {len(parts)})"
            except:
                format_results['face_detections'] = "❌ Parse error"
        else:
            format_results['face_detections'] = "⚠️  No messages to validate"
        
        # 顔認識メッセージ検証
        if '/face_identities' in results['topics'] and results['topics']['/face_identities']['latest_message']:
            try:
                msg = results['topics']['/face_identities']['latest_message']
                parts = msg.split('|')
                if len(parts) == 7:
                    format_results['face_identities'] = "✅ Valid format"
                else:
                    format_results['face_identities'] = f"❌ Invalid format (expected 7 parts, got {len(parts)})"
            except:
                format_results['face_identities'] = "❌ Parse error"
        else:
            format_results['face_identities'] = "⚠️  No messages to validate"
        
        # 注視状態メッセージ検証
        if '/gaze_status' in results['topics'] and results['topics']['/gaze_status']['latest_message']:
            try:
                msg = results['topics']['/gaze_status']['latest_message']
                parts = msg.split('|')
                if len(parts) == 6 and parts[1] in ['TRACKING', 'ENGAGED', 'DISENGAGED']:
                    format_results['gaze_status'] = "✅ Valid format"
                else:
                    format_results['gaze_status'] = f"❌ Invalid format or status"
            except:
                format_results['gaze_status'] = "❌ Parse error"
        else:
            format_results['gaze_status'] = "⚠️  No messages to validate"
        
        # フォーマット検証結果表示
        for topic, result in format_results.items():
            print(f"  {topic:20} {result}")
    
    def run_complete_test(self):
        """完全なテストを実行"""
        success = False
        
        try:
            # テスト画像の確認
            test_image = "test_images/person1_test.jpg"
            if not os.path.exists(test_image):
                self.get_logger().error(f"Test image not found: {test_image}")
                self.get_logger().info("Please run: python scripts/generate_test_images.py")
                return False
            
            print("🚀 Starting Reliable Pipeline Validation Test")
            print(f"📷 Using test image: {test_image}")
            print("⏱️  This test will run for 20 seconds..\\n")
            
            # テストカメラ起動
            if not self.start_test_camera(test_image, fps=3.0):
                return False
            
            # カメラ起動待機
            time.sleep(3)
            
            # 検証テスト実行
            results = self.run_validation_test(duration=20)
            
            # 結果表示
            self.print_detailed_results(results)
            
            success = results['overall_status'] in ['PASS', 'PARTIAL']
            
        except KeyboardInterrupt:
            print("\\n⚠️  Test interrupted by user")
        except Exception as e:
            self.get_logger().error(f"Test failed with error: {e}")
        finally:
            # クリーンアップ
            self.stop_test_camera()
        
        return success

def main():
    rclpy.init()
    
    try:
        validator = ReliablePipelineValidator()
        success = validator.run_complete_test()
        
        if success:
            print("\\n🎉 Pipeline validation completed successfully!")
            sys.exit(0)
        else:
            print("\\n❌ Pipeline validation failed!")
            sys.exit(1)
            
    except Exception as e:
        print(f"\\n💥 Validation script failed: {e}")
        sys.exit(1)
    finally:
        if 'validator' in locals():
            validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()