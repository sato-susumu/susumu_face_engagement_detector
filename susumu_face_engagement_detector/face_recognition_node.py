import os
from typing import List
import face_recognition
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Duration


class FaceIdentifier:
    def __init__(self, known_dir: str, tolerance: float) -> None:
        self._tolerance = tolerance
        self._known_encodings: List[np.ndarray] = []
        self._known_ids: List[str] = []
        self._tracked_encodings: List[np.ndarray] = []
        self._tracked_ids: List[str] = []
        self._next_id = 1
        self._load_known(known_dir)

    def _load_known(self, directory: str) -> None:
        if not os.path.exists(directory):
            return
        for fname in os.listdir(directory):
            path = os.path.join(directory, fname)
            image = face_recognition.load_image_file(path)
            encs = face_recognition.face_encodings(image)
            if encs:
                self._known_encodings.append(encs[0])
                self._known_ids.append(os.path.splitext(fname)[0])

    def identify(self, encoding: np.ndarray) -> str:
        for idx, enc in enumerate(self._known_encodings):
            if face_recognition.compare_faces([enc], encoding, tolerance=self._tolerance)[0]:
                return self._known_ids[idx]
        for idx, enc in enumerate(self._tracked_encodings):
            if face_recognition.compare_faces([enc], encoding, tolerance=self._tolerance)[0]:
                return self._tracked_ids[idx]
        new_id = f'user_{self._next_id}'
        self._next_id += 1
        self._tracked_encodings.append(encoding)
        self._tracked_ids.append(new_id)
        return new_id


class FaceRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__('face_recognition_node')
        self._declare_params()
        self._identifier = FaceIdentifier(
            known_dir=self.get_parameter('known_faces_dir').value,
            tolerance=self.get_parameter('match_tolerance').value
        )
        
        self.create_subscription(String, 'face_detections', self._on_face_detection, qos_profile=10)
        self._face_identity_pub = self.create_publisher(String, 'face_identities', qos_profile=10)
        
        # RViz可視化用パブリッシャー
        self._identity_markers_pub = self.create_publisher(MarkerArray, 'face_identity_markers', qos_profile=10)
        
        # 顔認識用の色設定（既知・未知・ユーザー別）
        self._identity_colors = {
            'known': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),     # 緑：既知の人物
            'unknown': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),   # 赤：未知の人物
            'user_1': ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9),    # 青：ユーザー1
            'user_2': ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9),    # 黄：ユーザー2
            'user_3': ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9),    # マゼンタ：ユーザー3
            'user_4': ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9),    # シアン：ユーザー4
            'user_5': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),    # オレンジ：ユーザー5
            'default': ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.9),   # グレー：その他
        }
        
        self.get_logger().info('Face Recognition Node started - Input topic: /face_detections')
        self.get_logger().info('Face Recognition Node - Output topics: /face_identities, /face_identity_markers')

    def _declare_params(self) -> None:
        for name, default in [
            ('known_faces_dir', 'known_faces'),
            ('match_tolerance', 0.6)
        ]:
            self.declare_parameter(name, default)

    def _on_face_detection(self, msg: String) -> None:
        data = msg.data.split('|')
        if len(data) < 8:
            return
            
        face_idx = data[0]
        center_x = float(data[1])
        center_y = float(data[2])
        width = float(data[3])
        height = float(data[4])
        frame_width = int(data[5])
        frame_height = int(data[6])
        encoding_str = data[7]
        
        encoding = np.array([float(x) for x in encoding_str.split(',')])
        face_id = self._identifier.identify(encoding)
        
        # 既存の文字列データ出力
        identity_data = f"{face_id}|{center_x}|{center_y}|{width}|{height}|{frame_width}|{frame_height}"
        identity_msg = String(data=identity_data)
        self._face_identity_pub.publish(identity_msg)
        
        # RViz用マーカーを作成・公開
        marker_array = MarkerArray()
        
        # 顔認識マーカー（球体）
        identity_marker = self._create_identity_marker(
            face_id, center_x, center_y, width, height, frame_width, frame_height
        )
        marker_array.markers.append(identity_marker)
        
        # テキストマーカー（顔ID表示）
        text_marker = self._create_text_marker(
            face_id, center_x, center_y, frame_width, frame_height
        )
        marker_array.markers.append(text_marker)
        
        self._identity_markers_pub.publish(marker_array)
    
    def _get_identity_color(self, face_id: str) -> ColorRGBA:
        """顔IDに対応する色を取得"""
        # 既知の人物かチェック
        if face_id in [known_id for known_id in self._identifier._known_ids]:
            return self._identity_colors['known']
        
        # ユーザーIDに対応する色
        if face_id in self._identity_colors:
            return self._identity_colors[face_id]
            
        return self._identity_colors['default']
    
    def _create_identity_marker(self, face_id: str, center_x: float, center_y: float,
                               width: float, height: float, frame_width: int, frame_height: int) -> Marker:
        """顔認識結果用のマーカーを作成"""
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_identities"
        marker.id = hash(face_id) % 1000  # face_idをハッシュ化してIDに
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 位置設定（顔検出位置より少し奥に配置）
        marker.pose.position.x = 1.2  # カメラから1.2m前方
        marker.pose.position.y = -(center_x - frame_width/2) * 0.002
        marker.pose.position.z = -(center_y - frame_height/2) * 0.002
        marker.pose.orientation.w = 1.0
        
        # サイズ設定（顔検出マーカーより大きく）
        scale = max(width, height) * 0.0012
        marker.scale = Vector3(x=scale, y=scale, z=scale)
        
        # 色設定
        marker.color = self._get_identity_color(face_id)
        
        # 生存時間
        marker.lifetime = Duration(sec=2, nanosec=0)
        
        return marker
    
    def _create_text_marker(self, face_id: str, center_x: float, center_y: float,
                           frame_width: int, frame_height: int) -> Marker:
        """顔ID表示用のテキストマーカーを作成"""
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_identity_text"
        marker.id = hash(face_id) % 1000 + 1000  # テキスト用に別のID範囲
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 位置設定（顔の上部に配置）
        marker.pose.position.x = 1.15
        marker.pose.position.y = -(center_x - frame_width/2) * 0.002
        marker.pose.position.z = -(center_y - frame_height/2) * 0.002 + 0.1  # 少し上
        marker.pose.orientation.w = 1.0
        
        # テキスト設定
        marker.text = face_id
        marker.scale.z = 0.08  # テキストサイズ
        
        # 色設定
        marker.color = self._get_identity_color(face_id)
        
        # 生存時間
        marker.lifetime = Duration(sec=2, nanosec=0)
        
        return marker


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()