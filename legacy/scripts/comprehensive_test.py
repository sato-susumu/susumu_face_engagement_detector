#!/usr/bin/env python3
"""
包括的テストスクリプト - 様々なシナリオでの動作検証
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import signal
import os
import sys
from collections import defaultdict

class ComprehensiveTestRunner(Node):
    def __init__(self):
        super().__init__('comprehensive_test_runner')
        self.test_results = {}
        self.current_process = None
        
        self.get_logger().info('Comprehensive Test Runner started')
    
    def run_test_scenario(self, scenario_name, camera_args, duration=15):
        """テストシナリオを実行"""
        self.get_logger().info(f'Starting test: {scenario_name}')
        
        # テスト用カメラ起動
        cmd = [
            'python', 'susumu_face_engagement_detector/test_camera_node.py',
            '--ros-args'] + camera_args
        
        try:
            # プロセス起動
            process = subprocess.Popen(cmd, 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE,
                                     text=True)
            self.current_process = process
            
            # 指定時間待機
            time.sleep(duration)
            
            # プロセス終了
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            
            # 結果収集
            result = self.collect_test_results(scenario_name)
            self.test_results[scenario_name] = result
            
            self.get_logger().info(f'Completed test: {scenario_name}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'Test failed: {scenario_name} - {e}')
            return {'status': 'FAILED', 'error': str(e)}
    
    def collect_test_results(self, scenario_name):
        """テスト結果を収集"""
        results = {
            'scenario': scenario_name,
            'status': 'UNKNOWN',
            'topics_active': [],
            'topics_inactive': [],
            'message_counts': {},
            'errors': []
        }
        
        # トピック確認
        topics_to_check = [
            '/camera/color/image_raw',
            '/face_detections', 
            '/face_identities',
            '/gaze_status',
            '/face_event',
            '/gaze_event'
        ]
        
        for topic in topics_to_check:
            try:
                # トピックの存在確認
                topic_list = subprocess.check_output(['ros2', 'topic', 'list'], 
                                                   text=True, timeout=5)
                if topic in topic_list:
                    # メッセージ確認
                    try:
                        msg = subprocess.check_output([
                            'ros2', 'topic', 'echo', topic, '--once'
                        ], text=True, timeout=3)
                        
                        if msg.strip():
                            results['topics_active'].append(topic)
                        else:
                            results['topics_inactive'].append(topic)
                    except subprocess.TimeoutExpired:
                        results['topics_inactive'].append(topic)
                else:
                    results['topics_inactive'].append(topic)
                    
            except Exception as e:
                results['errors'].append(f'Topic check failed for {topic}: {e}')
        
        # 結果判定
        active_count = len(results['topics_active'])
        total_count = len(topics_to_check)
        
        if active_count == total_count:
            results['status'] = 'PASS'
        elif active_count >= total_count * 0.8:
            results['status'] = 'PARTIAL'
        else:
            results['status'] = 'FAIL'
        
        results['activity_rate'] = active_count / total_count
        
        return results
    
    def run_all_tests(self):
        """全テストシナリオを実行"""
        test_scenarios = [
            {
                'name': 'Synthetic Dynamic Face',
                'args': ['-p', 'test_mode:=synthetic', '-p', 'fps:=3.0'],
                'duration': 10
            },
            {
                'name': 'Static Single Face',
                'args': ['-p', 'test_mode:=file', 
                        '-p', 'image_file:=test_images/person1_test.jpg',
                        '-p', 'fps:=2.0'],
                'duration': 8
            },
            {
                'name': 'Static Multiple Faces',
                'args': ['-p', 'test_mode:=file',
                        '-p', 'image_file:=test_images/multiple_faces_test.jpg',
                        '-p', 'fps:=2.0'],
                'duration': 8
            },
            {
                'name': 'No Face Image',
                'args': ['-p', 'test_mode:=file',
                        '-p', 'image_file:=test_images/no_face_test.jpg',
                        '-p', 'fps:=2.0'],
                'duration': 6
            },
            {
                'name': 'High FPS Test',
                'args': ['-p', 'test_mode:=synthetic', '-p', 'fps:=8.0'],
                'duration': 5
            },
            {
                'name': 'Low FPS Test',
                'args': ['-p', 'test_mode:=synthetic', '-p', 'fps:=1.0'],
                'duration': 8
            },
            {
                'name': 'Different Resolution',
                'args': ['-p', 'test_mode:=file',
                        '-p', 'image_file:=test_images/test_face_800x600.jpg',
                        '-p', 'fps:=2.0'],
                'duration': 6
            }
        ]
        
        # 各テスト実行
        for scenario in test_scenarios:
            try:
                # テスト間の待機時間
                time.sleep(2)
                
                result = self.run_test_scenario(
                    scenario['name'], 
                    scenario['args'], 
                    scenario['duration']
                )
                
                # 結果表示
                self.print_test_result(result)
                
            except KeyboardInterrupt:
                self.get_logger().info('Test interrupted by user')
                break
            except Exception as e:
                self.get_logger().error(f'Test scenario failed: {e}')
        
        # 最終結果表示
        self.print_final_results()
    
    def print_test_result(self, result):
        """個別テスト結果を表示"""
        print(f\"\\n{'='*60}\")
        print(f\"Test: {result['scenario']}\")
        print(f\"Status: {result['status']}\")
        print(f\"Activity Rate: {result['activity_rate']:.1%}\")
        
        if result['topics_active']:
            print(f\"✅ Active Topics: {', '.join(result['topics_active'])}\")
        
        if result['topics_inactive']:
            print(f\"❌ Inactive Topics: {', '.join(result['topics_inactive'])}\")
        
        if result['errors']:
            print(f\"⚠️ Errors: {'; '.join(result['errors'])}\")
    
    def print_final_results(self):
        """最終結果サマリーを表示"""
        print(f\"\\n{'='*80}\")
        print(\"COMPREHENSIVE TEST RESULTS SUMMARY\")
        print(f\"{'='*80}\")
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for r in self.test_results.values() if r['status'] == 'PASS')
        partial_tests = sum(1 for r in self.test_results.values() if r['status'] == 'PARTIAL')
        failed_tests = sum(1 for r in self.test_results.values() if r['status'] == 'FAIL')
        
        print(f\"\\n📊 Test Summary:\")
        print(f\"  Total Tests:   {total_tests}\")
        print(f\"  ✅ Passed:     {passed_tests}\")
        print(f\"  🟡 Partial:    {partial_tests}\")
        print(f\"  ❌ Failed:     {failed_tests}\")
        print(f\"  Success Rate:  {(passed_tests + partial_tests)/total_tests:.1%}\")
        
        print(f\"\\n📋 Detailed Results:\")
        for name, result in self.test_results.items():
            status_icon = {'PASS': '✅', 'PARTIAL': '🟡', 'FAIL': '❌'}.get(result['status'], '❓')
            print(f\"  {status_icon} {name:25} {result['status']:8} ({result['activity_rate']:.1%})\")
        
        # 推奨事項
        print(f\"\\n💡 Recommendations:\")
        if failed_tests == 0:
            print(\"  🎉 All tests passed! System is working excellently.\")
        elif failed_tests <= total_tests * 0.2:
            print(\"  👍 Most tests passed. Minor issues may need attention.\")
        else:
            print(\"  ⚠️ Multiple test failures detected. System needs investigation.\")
            
        # 最も問題のあるトピック
        topic_failures = defaultdict(int)
        for result in self.test_results.values():
            for topic in result['topics_inactive']:
                topic_failures[topic] += 1
        
        if topic_failures:
            print(f\"\\n🔍 Most Problematic Topics:\")
            for topic, count in sorted(topic_failures.items(), key=lambda x: x[1], reverse=True)[:3]:
                print(f\"  {topic}: {count}/{total_tests} failures\")

def main():
    rclpy.init()
    
    try:
        runner = ComprehensiveTestRunner()
        
        print(\"Starting Comprehensive Face Engagement Detection Test\")
        print(\"This will run multiple test scenarios automatically.\")
        print(\"Press Ctrl+C to interrupt at any time.\\n\")
        
        # 少し待機して準備
        time.sleep(2)
        
        runner.run_all_tests()
        
    except KeyboardInterrupt:
        print(\"\\nTest interrupted by user\")
    except Exception as e:
        print(f\"Test runner failed: {e}\")
    finally:
        if 'runner' in locals():
            runner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()