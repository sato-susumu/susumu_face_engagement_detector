#!/usr/bin/env python3

import os
import psutil
import subprocess
import time
import signal
import sys
from typing import List, Dict

class ProcessCountVerifier:
    """
    1プロセス vs 複数プロセスでの動作を検証するクラス
    """
    
    def __init__(self):
        self.test_results = {}
        self.processes = []
        
    def get_face_engagement_processes(self) -> List[Dict]:
        """顔エンゲージメント関連のプロセスを取得"""
        processes = []
        keywords = ['face_detection', 'face_recognition', 'gaze_analysis', 'engagement_manager', 'multi_node_executor']
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                if any(keyword in cmdline for keyword in keywords):
                    processes.append({
                        'pid': proc.info['pid'],
                        'name': proc.info['name'],
                        'cmdline': cmdline
                    })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
                
        return processes
        
    def cleanup_processes(self):
        """既存のプロセスをクリーンアップ"""
        print("🧹 Cleaning up existing processes...")
        processes = self.get_face_engagement_processes()
        
        for proc_info in processes:
            try:
                proc = psutil.Process(proc_info['pid'])
                proc.terminate()
                proc.wait(timeout=3)
                print(f"  ✅ Terminated: {proc_info['name']} (PID: {proc_info['pid']})")
            except (psutil.NoSuchProcess, psutil.TimeoutExpired):
                pass
        
        time.sleep(1)
        
    def start_launch_file(self, launch_file: str, timeout: int = 10) -> subprocess.Popen:
        """ランチファイルを起動"""
        cmd = ['ros2', 'launch', 'susumu_face_engagement_detector', launch_file]
        print(f"🚀 Starting: {' '.join(cmd)}")
        
        proc = subprocess.Popen(
            cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        # 起動待機
        time.sleep(timeout)
        return proc
        
    def test_single_process_mode(self) -> Dict:
        """1プロセスモードテスト"""
        print("\n" + "="*80)
        print("🔧 Testing Single Process Mode (multi_node_launch.py)")
        print("="*80)
        
        self.cleanup_processes()
        
        # multi_node_executorを直接実行
        print("🚀 Starting multi_node_executor directly...")
        proc = subprocess.Popen(
            ['python', 'susumu_face_engagement_detector/multi_node_executor.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        time.sleep(8)  # 起動待機
        
        # プロセス数確認
        processes = self.get_face_engagement_processes()
        process_count = len(processes)
        
        # 結果記録
        result = {
            'mode': 'Single Process',
            'launch_method': 'multi_node_executor.py',
            'process_count': process_count,
            'processes': processes,
            'expected_count': 1,
            'success': process_count == 1
        }
        
        print(f"📊 Process Count: {process_count}")
        print(f"✅ Expected: 1 process")
        print(f"🎯 Result: {'PASS' if result['success'] else 'FAIL'}")
        
        if processes:
            print("\n📋 Detected Processes:")
            for i, proc in enumerate(processes, 1):
                print(f"  {i}. PID: {proc['pid']}, Name: {proc['name']}")
                print(f"     Command: {proc['cmdline'][:100]}...")
        
        # プロセス終了
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3)
        except:
            pass
            
        time.sleep(2)
        return result
        
    def test_multi_process_mode(self) -> Dict:
        """複数プロセスモードテスト"""
        print("\n" + "="*80)
        print("🔧 Testing Multi Process Mode (simple_launch.py)")
        print("="*80)
        
        self.cleanup_processes()
        
        # simple_launch.pyを実行
        proc = self.start_launch_file('simple_launch.py', timeout=8)
        
        # プロセス数確認
        processes = self.get_face_engagement_processes()
        process_count = len(processes)
        
        # 結果記録
        result = {
            'mode': 'Multi Process',
            'launch_method': 'simple_launch.py',
            'process_count': process_count,
            'processes': processes,
            'expected_count': 4,
            'success': process_count == 4
        }
        
        print(f"📊 Process Count: {process_count}")
        print(f"✅ Expected: 4 processes")
        print(f"🎯 Result: {'PASS' if result['success'] else 'FAIL'}")
        
        if processes:
            print("\n📋 Detected Processes:")
            for i, proc in enumerate(processes, 1):
                print(f"  {i}. PID: {proc['pid']}, Name: {proc['name']}")
                print(f"     Command: {proc['cmdline'][:100]}...")
        
        # プロセス終了
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3)
        except:
            pass
            
        time.sleep(2)
        return result
        
    def run_verification(self) -> Dict:
        """検証実行"""
        print("🔍 Face Engagement Detection System - Process Count Verification")
        print(f"📅 Test Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        # テスト実行
        single_result = self.test_single_process_mode()
        multi_result = self.test_multi_process_mode()
        
        # 最終クリーンアップ
        self.cleanup_processes()
        
        # 結果サマリー
        results = {
            'single_process': single_result,
            'multi_process': multi_result,
            'overall_success': single_result['success'] and multi_result['success']
        }
        
        self.print_summary(results)
        return results
        
    def print_summary(self, results: Dict):
        """結果サマリー表示"""
        print("\n" + "="*80)
        print("📊 VERIFICATION SUMMARY")
        print("="*80)
        
        print(f"\n🔧 Single Process Mode:")
        print(f"  Method: {results['single_process']['launch_method']}")
        print(f"  Process Count: {results['single_process']['process_count']} (Expected: 1)")
        print(f"  Result: {'✅ PASS' if results['single_process']['success'] else '❌ FAIL'}")
        
        print(f"\n🔧 Multi Process Mode:")
        print(f"  Method: {results['multi_process']['launch_method']}")
        print(f"  Process Count: {results['multi_process']['process_count']} (Expected: 4)")
        print(f"  Result: {'✅ PASS' if results['multi_process']['success'] else '❌ FAIL'}")
        
        overall = "✅ ALL TESTS PASSED" if results['overall_success'] else "❌ SOME TESTS FAILED"
        print(f"\n🎯 Overall Result: {overall}")
        
        if results['overall_success']:
            print("\n💡 Both single-process and multi-process modes are working correctly!")
            print("   - Use multi_node_executor.py for single-process efficiency")
            print("   - Use simple_launch.py for multi-process isolation")
        else:
            print("\n⚠️  Some issues detected. Please check the process counts above.")
            
        print("="*80)


def main():
    """メイン関数"""
    try:
        verifier = ProcessCountVerifier()
        results = verifier.run_verification()
        return 0 if results['overall_success'] else 1
        
    except KeyboardInterrupt:
        print("\n⏹️  Verification interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Error during verification: {e}")
        return 1


if __name__ == '__main__':
    exit(main())