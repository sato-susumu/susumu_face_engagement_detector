#!/usr/bin/env python3
"""
顔エンゲージメント検出システムのパイプライン検証スクリプト
"""

import rclpy
from rclpy.node import Node
import time
import threading
import sys
from collections import defaultdict, deque
from std_msgs.msg import String
from sensor_msgs.msg import Image

class PipelineValidator(Node):
    def __init__(self):
        super().__init__('pipeline_validator')
        
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
        
        # 検証結果
        self.validation_results = {}
        
        # 各トピックの購読設定
        self.setup_subscriptions()
        
        # 検証タイマー
        self.validation_timer = self.create_timer(2.0, self.run_validation)
        self.test_duration = 30.0  # 30秒間テスト
        self.start_time = time.time()
        
        self.get_logger().info('Pipeline Validator started')
        self.get_logger().info('Monitoring pipeline for 30 seconds...')
    
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
            
            # 最新の10メッセージのみ保持
            if len(self.messages[topic_name]) > 10:
                self.messages[topic_name].pop(0)
                self.message_times[topic_name].pop(0)
                
        return callback
    
    def run_validation(self):
        """パイプライン検証実行"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time > self.test_duration:
            self.print_final_results()
            rclpy.shutdown()
            return
        
        # 現在の状況を出力
        self.print_current_status(elapsed_time)
    
    def print_current_status(self, elapsed_time):
        """現在の状況を表示"""
        print(f"\n{'='*60}")
        print(f"Pipeline Validation - {elapsed_time:.1f}s elapsed")
        print(f"{'='*60}")
        
        for topic_name in self.topics.keys():
            msg_count = len(self.messages[topic_name])
            
            if msg_count > 0:
                latest_msg = self.messages[topic_name][-1]
                
                # Hz計算
                if len(self.message_times[topic_name]) >= 2:
                    time_window = self.message_times[topic_name][-1] - self.message_times[topic_name][0]
                    hz = (len(self.message_times[topic_name]) - 1) / time_window if time_window > 0 else 0
                else:
                    hz = 0
                
                status = f"✅ ACTIVE ({msg_count} msgs, {hz:.1f} Hz)"
                if topic_name != '/camera/color/image_raw':
                    status += f"\n   Latest: {latest_msg[:80]}{'...' if len(str(latest_msg)) > 80 else ''}"
            else:
                status = "❌ NO MESSAGES"
            
            print(f"{topic_name:25} {status}")
    
    def analyze_pipeline_flow(self):
        """パイプラインフロー分析"""
        results = {}
        
        # 各段階の評価
        camera_msgs = len(self.messages['/camera/color/image_raw'])
        detection_msgs = len(self.messages['/face_detections'])
        recognition_msgs = len(self.messages['/face_identities'])
        gaze_msgs = len(self.messages['/gaze_status'])
        face_events = len(self.messages['/face_event'])
        gaze_events = len(self.messages['/gaze_event'])
        
        results['camera_input'] = camera_msgs > 0
        results['face_detection'] = detection_msgs > 0
        results['face_recognition'] = recognition_msgs > 0
        results['gaze_analysis'] = gaze_msgs > 0
        results['event_generation'] = face_events > 0 or gaze_events > 0
        
        # パイプライン効率
        if camera_msgs > 0:
            results['detection_efficiency'] = detection_msgs / camera_msgs
            results['recognition_efficiency'] = recognition_msgs / camera_msgs if camera_msgs > 0 else 0
        else:
            results['detection_efficiency'] = 0
            results['recognition_efficiency'] = 0
        
        return results
    
    def validate_message_formats(self):
        """メッセージフォーマット検証"""
        format_results = {}
        
        # 顔検出メッセージ検証
        if self.messages['/face_detections']:
            valid_detections = 0
            for msg in self.messages['/face_detections']:
                try:
                    parts = msg.split('|')
                    if len(parts) == 8:  # face_idx|center_x|center_y|width|height|frame_width|frame_height|feature_csv
                        valid_detections += 1
                except:
                    pass
            
            format_results['face_detections'] = {
                'total': len(self.messages['/face_detections']),
                'valid': valid_detections,
                'validity_rate': valid_detections / len(self.messages['/face_detections'])
            }
        
        # 顔認識メッセージ検証
        if self.messages['/face_identities']:
            valid_identities = 0
            for msg in self.messages['/face_identities']:
                try:
                    parts = msg.split('|')
                    if len(parts) == 7:  # face_id|center_x|center_y|width|height|frame_width|frame_height
                        valid_identities += 1
                except:
                    pass
            
            format_results['face_identities'] = {
                'total': len(self.messages['/face_identities']),
                'valid': valid_identities,
                'validity_rate': valid_identities / len(self.messages['/face_identities'])
            }
        
        return format_results
    
    def print_final_results(self):
        """最終結果を表示"""
        print(f"\n{'='*80}")
        print("FINAL VALIDATION RESULTS")
        print(f"{'='*80}")
        
        # パイプライン分析
        pipeline_results = self.analyze_pipeline_flow()
        
        print("\n📊 Pipeline Flow Analysis:")
        print("-" * 50)
        stages = [
            ('Camera Input', pipeline_results['camera_input']),
            ('Face Detection', pipeline_results['face_detection']),
            ('Face Recognition', pipeline_results['face_recognition']),
            ('Gaze Analysis', pipeline_results['gaze_analysis']),
            ('Event Generation', pipeline_results['event_generation'])
        ]
        
        for stage_name, passed in stages:
            status = "✅ PASS" if passed else "❌ FAIL"
            print(f"  {stage_name:20} {status}")
        
        # 効率分析
        print(f"\n⚡ Efficiency Analysis:")
        print("-" * 50)
        print(f"  Detection Efficiency:  {pipeline_results['detection_efficiency']:.2%}")
        print(f"  Recognition Efficiency: {pipeline_results['recognition_efficiency']:.2%}")
        
        # メッセージ統計
        print(f"\n📈 Message Statistics:")
        print("-" * 50)
        for topic_name in self.topics.keys():
            count = len(self.messages[topic_name])
            print(f"  {topic_name:25} {count:5} messages")
        
        # フォーマット検証
        format_results = self.validate_message_formats()
        if format_results:
            print(f"\n📝 Message Format Validation:")
            print("-" * 50)
            for topic, result in format_results.items():
                print(f"  {topic}:")
                print(f"    Total: {result['total']}, Valid: {result['valid']}")
                print(f"    Validity Rate: {result['validity_rate']:.1%}")
        
        # 最新メッセージサンプル
        print(f"\n📋 Latest Message Samples:")
        print("-" * 50)
        for topic_name in ['/face_detections', '/face_identities', '/gaze_status', '/face_event', '/gaze_event']:
            if self.messages[topic_name]:
                latest = self.messages[topic_name][-1]
                print(f"  {topic_name}:")
                print(f"    {latest[:100]}{'...' if len(latest) > 100 else ''}")
        
        # 総合評価
        total_stages = len(stages)
        passed_stages = sum(1 for _, passed in stages if passed)
        
        print(f"\n🎯 Overall Assessment:")
        print("-" * 50)
        print(f"  Stages Passed: {passed_stages}/{total_stages}")
        print(f"  Success Rate:  {passed_stages/total_stages:.1%}")
        
        if passed_stages == total_stages:
            print("  🟢 PIPELINE FULLY FUNCTIONAL")
        elif passed_stages >= total_stages * 0.8:
            print("  🟡 PIPELINE MOSTLY FUNCTIONAL")
        else:
            print("  🔴 PIPELINE NEEDS ATTENTION")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        validator = PipelineValidator()
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
    finally:
        if 'validator' in locals():
            validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()