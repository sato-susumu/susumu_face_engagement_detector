import os
from typing import List
import face_recognition
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String


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
        
        self.get_logger().info('Face Recognition Node started - Input topic: /face_detections')
        self.get_logger().info('Face Recognition Node - Output topic: /face_identities')

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
        center_x = data[1]
        center_y = data[2]
        width = data[3]
        height = data[4]
        frame_width = data[5]
        frame_height = data[6]
        encoding_str = data[7]
        
        encoding = np.array([float(x) for x in encoding_str.split(',')])
        face_id = self._identifier.identify(encoding)
        
        identity_data = f"{face_id}|{center_x}|{center_y}|{width}|{height}|{frame_width}|{frame_height}"
        identity_msg = String(data=identity_data)
        self._face_identity_pub.publish(identity_msg)


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