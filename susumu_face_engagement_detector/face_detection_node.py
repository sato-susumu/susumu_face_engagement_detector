from typing import List, Tuple
import cv2
import face_recognition
from cv_bridge import CvBridge
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3


class FaceDetector:
    def __init__(self, model: str) -> None:
        self._model = model

    def detect(self, rgb_frame: np.ndarray) -> Tuple[List[Tuple[int,int,int,int]], List[np.ndarray]]:
        locations = face_recognition.face_locations(rgb_frame, model=self._model)
        encodings = face_recognition.face_encodings(rgb_frame, locations)
        return locations, encodings


class FaceDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('face_detection_node')
        self._declare_params()
        self._bridge = CvBridge()
        self._detector = FaceDetector(self.get_parameter('detection_model').value)
        
        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self._on_image, qos_profile=10)
        self._face_detection_pub = self.create_publisher(String, 'face_detections', qos_profile=10)
        
        # RViz可視化用パブリッシャー
        self._face_markers_pub = self.create_publisher(MarkerArray, 'face_detection_markers', qos_profile=10)
        self._annotated_image_pub = self.create_publisher(Image, 'face_detection_image', qos_profile=10)
        
        # 色設定（複数人用）
        self._face_colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # 赤
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),  # 緑
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),  # 青
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),  # 黄
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),  # マゼンタ
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8),  # シアン
            ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),  # オレンジ
            ColorRGBA(r=0.5, g=0.0, b=1.0, a=0.8),  # 紫
        ]
        
        self.get_logger().info(f'Face Detection Node started - Input topic: {topic}')
        self.get_logger().info('Face Detection Node - Output topics: /face_detections, /face_detection_markers, /face_detection_image')

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/camera/color/image_raw'),
            ('detection_model', 'hog')
        ]:
            self.declare_parameter(name, default)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        locations, encodings = self._detector.detect(rgb)
        
        # アノテーション付き画像を作成
        annotated_frame = frame.copy()
        
        # RViz用マーカー配列
        marker_array = MarkerArray()
        
        for i, (location, encoding) in enumerate(zip(locations, encodings)):
            top, right, bottom, left = location
            center_x = (left + right) / 2
            center_y = (top + bottom) / 2
            width = right - left
            height = bottom - top
            
            # 既存の文字列データ出力
            encoding_str = ','.join(map(str, encoding))
            detection_data = f"{i}|{center_x}|{center_y}|{width}|{height}|{frame.shape[1]}|{frame.shape[0]}|{encoding_str}"
            detection_msg = String(data=detection_data)
            self._face_detection_pub.publish(detection_msg)
            
            # 画像にバウンディングボックスとラベルを描画
            color = self._get_face_color_bgr(i)
            cv2.rectangle(annotated_frame, (left, top), (right, bottom), color, 2)
            cv2.putText(annotated_frame, f'Face {i}', (left, top - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # RViz用マーカーを作成
            marker = self._create_face_marker(i, center_x, center_y, width, height, 
                                            frame.shape[1], frame.shape[0], msg.header)
            marker_array.markers.append(marker)
        
        # アノテーション付き画像を公開
        try:
            annotated_msg = self._bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            annotated_msg.header = msg.header
            self._annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish annotated image: {e}')
        
        # マーカー配列を公開
        self._face_markers_pub.publish(marker_array)
    
    def _get_face_color_bgr(self, face_index: int) -> Tuple[int, int, int]:
        """顔インデックスに対応するBGR色を取得"""
        color_rgba = self._face_colors[face_index % len(self._face_colors)]
        return (int(color_rgba.b * 255), int(color_rgba.g * 255), int(color_rgba.r * 255))
    
    def _create_face_marker(self, face_id: int, center_x: float, center_y: float, 
                           width: float, height: float, frame_width: int, frame_height: int,
                           header: Header) -> Marker:
        """RViz用の顔検出マーカーを作成"""
        marker = Marker()
        marker.header = header
        marker.header.frame_id = "camera_link"  # カメラフレーム
        marker.ns = "face_detections"
        marker.id = face_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 位置設定（画像座標系からカメラ座標系に変換）
        # 簡易的な変換（実際のカメラパラメータに応じて調整が必要）
        marker.pose.position.x = 1.0  # カメラから1m前方
        marker.pose.position.y = -(center_x - frame_width/2) * 0.002  # 左右
        marker.pose.position.z = -(center_y - frame_height/2) * 0.002  # 上下
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # サイズ設定
        marker.scale = Vector3(x=width * 0.001, y=height * 0.001, z=0.05)
        
        # 色設定
        marker.color = self._face_colors[face_id % len(self._face_colors)]
        
        # 生存時間
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        
        return marker


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = FaceDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()