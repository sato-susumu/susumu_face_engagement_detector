#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import threading
from typing import Optional

try:
    from .face_detection_node import FaceDetectionNode
    from .face_recognition_node import FaceRecognitionNode
    from .gaze_analysis_node import GazeAnalysisNode
    from .engagement_manager_node import EngagementManagerNode
except ImportError:
    from face_detection_node import FaceDetectionNode
    from face_recognition_node import FaceRecognitionNode
    from gaze_analysis_node import GazeAnalysisNode
    from engagement_manager_node import EngagementManagerNode


class MultiNodeExecutor:
    """
    1つのプロセス内で複数のノードを効率的に実行するためのエグゼキューター
    """
    
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.nodes = []
        self.running = False
        
    def add_nodes(self, **params):
        """ノードを追加"""
        try:
            # ノードを作成（パラメータはノード内でdefaultから設定される）
            face_detection = FaceDetectionNode()
            face_recognition = FaceRecognitionNode()
            gaze_analysis = GazeAnalysisNode()
            engagement_manager = EngagementManagerNode()
            
            # ノードをエグゼキューターに追加
            self.nodes = [face_detection, face_recognition, gaze_analysis, engagement_manager]
            for node in self.nodes:
                self.executor.add_node(node)
                
            print(f"✅ Added {len(self.nodes)} nodes to single-process executor")
            print("📊 All nodes running in single Python process")
            
        except Exception as e:
            print(f"❌ Error adding nodes: {e}")
            raise
            
    def spin(self):
        """エグゼキューターを開始"""
        try:
            self.running = True
            print("🚀 Starting multi-node executor in single process...")
            print(f"📝 Process ID: {threading.get_ident()}")
            print("📊 Running nodes:")
            for node in self.nodes:
                print(f"  - {node.get_name()}")
            print("---")
            
            self.executor.spin()
            
        except KeyboardInterrupt:
            print("\n⏹️  Shutdown requested...")
        except Exception as e:
            print(f"❌ Error in executor: {e}")
        finally:
            self.shutdown()
            
    def shutdown(self):
        """エグゼキューターを停止"""
        if self.running:
            print("🛑 Shutting down multi-node executor...")
            self.running = False
            
            # ノードを停止
            for node in self.nodes:
                try:
                    node.destroy_node()
                except:
                    pass
                    
            self.executor.shutdown()
            

def main(args=None):
    """メイン関数"""
    if args is None:
        args = sys.argv
        
    # ROS2初期化
    rclpy.init(args=args)
    
    try:
        # パラメータ解析（簡易版）
        params = {}
        for arg in args:
            if ':=' in arg:
                key, value = arg.split(':=', 1)
                key = key.lstrip('-p').strip()
                # 型変換
                if value.lower() in ['true', 'false']:
                    value = value.lower() == 'true'
                elif value.replace('.', '').isdigit():
                    value = float(value) if '.' in value else int(value)
                params[key] = value
                
        # マルチノードエグゼキューター作成・実行
        executor = MultiNodeExecutor()
        executor.add_nodes(**params)
        executor.spin()
        
    except Exception as e:
        print(f"❌ Failed to start multi-node executor: {e}")
        return 1
    finally:
        # ROS2終了
        try:
            rclpy.shutdown()
        except:
            pass
            
    return 0


if __name__ == '__main__':
    exit(main())