#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory

class TestCameraNode(Node):
    def __init__(self):
        super().__init__('test_camera_node')
        
        # パラメータ宣言
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('test_mode', 'synthetic')  # 'synthetic' or 'file'
        self.declare_parameter('image_file', '')
        
        # パラメータ取得
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.test_mode = self.get_parameter('test_mode').get_parameter_value().string_value
        self.image_file = self.get_parameter('image_file').get_parameter_value().string_value
        
        # CV Bridge初期化
        self.bridge = CvBridge()
        
        # Publisher作成
        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        
        # タイマー作成
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_image)
        
        # 顔位置とアニメーション用変数
        self.frame_count = 0
        self.face_x = self.width // 4
        self.face_y = self.height // 2
        self.face_direction = 1
        self.face_size = 80
        
        # ファイルモード用の画像読み込み
        self.loaded_image = None
        if self.test_mode == 'file' and self.image_file:
            if os.path.exists(self.image_file):
                self.loaded_image = cv2.imread(self.image_file)
                if self.loaded_image is not None:
                    self.loaded_image = cv2.resize(self.loaded_image, (self.width, self.height))
                    self.get_logger().info(f'Loaded test image from: {self.image_file}')
                else:
                    self.get_logger().warn(f'Failed to load image: {self.image_file}')
            else:
                self.get_logger().warn(f'Image file not found: {self.image_file}')
        
        self.get_logger().info(f'Test Camera Node started')
        self.get_logger().info(f'Publishing to: {self.image_topic}')
        self.get_logger().info(f'Mode: {self.test_mode}, FPS: {self.fps}, Size: {self.width}x{self.height}')
    
    def generate_synthetic_face_image(self):
        """著作権フリーの合成顔画像を生成"""
        # 背景作成（グラデーション）
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # 背景グラデーション
        for y in range(self.height):
            color_value = int(50 + (y / self.height) * 100)
            img[y, :] = [color_value, color_value + 20, color_value + 40]
        
        # 顔の動きアニメーション
        self.face_x += self.face_direction * 2
        if self.face_x > self.width - self.face_size - 50:
            self.face_direction = -1
        elif self.face_x < 50:
            self.face_direction = 1
        
        # 顔の上下移動も追加
        face_y_offset = int(20 * np.sin(self.frame_count * 0.1))
        current_face_y = self.face_y + face_y_offset
        
        # 顔の輪郭（楕円）
        face_center = (self.face_x + self.face_size // 2, current_face_y)
        face_axes = (self.face_size // 2, int(self.face_size * 0.6))
        cv2.ellipse(img, face_center, face_axes, 0, 0, 360, (220, 180, 160), -1)
        
        # 目の位置計算
        eye_y = current_face_y - 15
        left_eye_x = self.face_x + 20
        right_eye_x = self.face_x + self.face_size - 20
        
        # 左目
        cv2.circle(img, (left_eye_x, eye_y), 8, (255, 255, 255), -1)
        cv2.circle(img, (left_eye_x + 2, eye_y), 4, (50, 50, 50), -1)
        
        # 右目
        cv2.circle(img, (right_eye_x, eye_y), 8, (255, 255, 255), -1)
        cv2.circle(img, (right_eye_x + 2, eye_y), 4, (50, 50, 50), -1)
        
        # 鼻
        nose_points = np.array([
            [self.face_x + self.face_size // 2, current_face_y - 5],
            [self.face_x + self.face_size // 2 - 5, current_face_y + 10],
            [self.face_x + self.face_size // 2 + 5, current_face_y + 10]
        ], np.int32)
        cv2.fillPoly(img, [nose_points], (200, 160, 140))
        
        # 口
        mouth_center = (self.face_x + self.face_size // 2, current_face_y + 25)
        mouth_width = 25
        mouth_height = 8
        
        # 微笑み効果
        smile_offset = int(5 * np.sin(self.frame_count * 0.05))
        cv2.ellipse(img, mouth_center, (mouth_width, mouth_height + smile_offset), 
                   0, 0, 180, (100, 50, 50), 2)
        
        # フレーム情報をテキストで表示
        cv2.putText(img, f'Frame: {self.frame_count}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f'Test Face: Moving', (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f'Position: ({self.face_x}, {current_face_y})', (10, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return img
    
    def publish_image(self):
        """画像をパブリッシュ"""
        try:
            if self.test_mode == 'file' and self.loaded_image is not None:
                # ファイルから読み込んだ画像を使用
                img = self.loaded_image.copy()
                
                # フレーム情報を追加
                cv2.putText(img, f'Frame: {self.frame_count}', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(img, f'File: {os.path.basename(self.image_file)}', (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                # 合成画像を生成
                img = self.generate_synthetic_face_image()
            
            # ROS2メッセージに変換
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            
            # パブリッシュ
            self.publisher.publish(img_msg)
            
            self.frame_count += 1
            
            # 100フレームごとにログ出力
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_camera = TestCameraNode()
        rclpy.spin(test_camera)
    except KeyboardInterrupt:
        print("\nShutting down test camera...")
    finally:
        if 'test_camera' in locals():
            test_camera.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()