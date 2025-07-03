#!/usr/bin/env python3
"""
テスト環境チェックスクリプト
テスト実行前に必要な条件が整っているかを確認
"""

import os
import sys
import subprocess
import rclpy

def check_test_images():
    """テスト画像の存在確認"""
    print("📷 Checking test images...")
    
    required_images = [
        "test_images/person1_test.jpg",
        "test_images/person2_test.jpg", 
        "test_images/person3_test.jpg",
        "test_images/multiple_faces_test.jpg",
        "test_images/no_face_test.jpg"
    ]
    
    missing_images = []
    for image_path in required_images:
        if os.path.exists(image_path):
            print(f"  ✅ {image_path}")
        else:
            print(f"  ❌ {image_path}")
            missing_images.append(image_path)
    
    if missing_images:
        print(f"\\n⚠️  Missing {len(missing_images)} test images")
        print("🔧 Run: python scripts/generate_test_images.py")
        return False
    else:
        print("✅ All test images found")
        return True

def check_ros2_nodes():
    """ROS2ノードが起動可能かチェック"""
    print("\\n🤖 Checking ROS2 nodes...")
    
    try:
        # パッケージの存在確認
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        if 'susumu_face_engagement_detector' in result.stdout:
            print("  ✅ Package found")
        else:
            print("  ❌ Package not found")
            print("🔧 Run: colcon build --packages-select susumu_face_engagement_detector")
            return False
        
        # 実行ファイルの確認
        result = subprocess.run(['ros2', 'pkg', 'executables', 'susumu_face_engagement_detector'], 
                              capture_output=True, text=True, timeout=5)
        
        required_executables = [
            'face_detection_node',
            'face_recognition_node', 
            'gaze_analysis_node',
            'engagement_manager_node',
            'monitoring_node'
        ]
        
        missing_executables = []
        for executable in required_executables:
            if executable in result.stdout:
                print(f"  ✅ {executable}")
            else:
                print(f"  ❌ {executable}")
                missing_executables.append(executable)
        
        if missing_executables:
            print(f"\\n⚠️  Missing {len(missing_executables)} executables")
            print("🔧 Run: colcon build --packages-select susumu_face_engagement_detector")
            return False
        else:
            print("✅ All executables found")
            return True
            
    except Exception as e:
        print(f"  ❌ ROS2 check failed: {e}")
        return False

def check_python_dependencies():
    """Python依存関係チェック"""
    print("\\n🐍 Checking Python dependencies...")
    
    required_modules = [
        'cv2',
        'numpy', 
        'face_recognition',
        'rclpy'
    ]
    
    missing_modules = []
    for module in required_modules:
        try:
            __import__(module)
            print(f"  ✅ {module}")
        except ImportError:
            print(f"  ❌ {module}")
            missing_modules.append(module)
    
    if missing_modules:
        print(f"\\n⚠️  Missing {len(missing_modules)} Python modules")
        print("🔧 Install missing modules:")
        if 'cv2' in missing_modules:
            print("   pip install opencv-python")
        if 'numpy' in missing_modules:
            print("   pip install numpy")
        if 'face_recognition' in missing_modules:
            print("   pip install face_recognition")
        if 'rclpy' in missing_modules:
            print("   sudo apt install ros-humble-rclpy")
        return False
    else:
        print("✅ All Python dependencies found")
        return True

def check_running_nodes():
    """実行中のノード確認"""
    print("\\n🔍 Checking running nodes...")
    
    try:
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        face_nodes = [line for line in result.stdout.split('\\n') 
                     if 'face' in line or 'gaze' in line or 'engagement' in line]
        
        if face_nodes:
            print("  ⚠️  Face engagement nodes already running:")
            for node in face_nodes:
                print(f"    - {node}")
            print("  💡 This may interfere with tests")
            return 'WARNING'
        else:
            print("  ✅ No conflicting nodes found")
            return True
            
    except Exception as e:
        print(f"  ⚠️  Node check failed: {e}")
        return 'WARNING'

def main():
    print("🔍 Face Engagement Detection System - Test Environment Check")
    print("="*60)
    
    checks = [
        ("Test Images", check_test_images),
        ("Python Dependencies", check_python_dependencies), 
        ("ROS2 Nodes", check_ros2_nodes),
        ("Running Nodes", check_running_nodes)
    ]
    
    all_passed = True
    warnings = False
    
    for check_name, check_func in checks:
        result = check_func()
        if result == False:
            all_passed = False
        elif result == 'WARNING':
            warnings = True
    
    print("\\n" + "="*60)
    print("ENVIRONMENT CHECK SUMMARY")
    print("="*60)
    
    if all_passed and not warnings:
        print("🎉 Environment check passed!")
        print("✅ Ready to run tests")
        print("\\n🚀 Suggested test commands:")
        print("   python scripts/simple_integration_test.py")
        print("   python scripts/test_pipeline.py")
        return True
    elif all_passed and warnings:
        print("🟡 Environment check passed with warnings")
        print("⚠️  Some issues detected but tests should work")
        return True
    else:
        print("❌ Environment check failed!")
        print("🔧 Please fix the issues above before running tests")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)