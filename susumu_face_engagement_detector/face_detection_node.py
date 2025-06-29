from typing import List, Tuple
import cv2
import face_recognition
from cv_bridge import CvBridge
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


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

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/image'),
            ('detection_model', 'hog')
        ]:
            self.declare_parameter(name, default)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        locations, encodings = self._detector.detect(rgb)
        
        for i, (location, encoding) in enumerate(zip(locations, encodings)):
            top, right, bottom, left = location
            center_x = (left + right) / 2
            center_y = (top + bottom) / 2
            width = right - left
            height = bottom - top
            
            encoding_str = ','.join(map(str, encoding))
            detection_data = f"{i}|{center_x}|{center_y}|{width}|{height}|{frame.shape[1]}|{frame.shape[0]}|{encoding_str}"
            msg = String(data=detection_data)
            self._face_detection_pub.publish(msg)


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