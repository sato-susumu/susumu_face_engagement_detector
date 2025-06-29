import os
import time
from typing import List, Tuple, Dict, Optional

import cv2
import face_recognition
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FaceDetector:
    def __init__(self, model: str) -> None:
        self._model = model

    def detect(self, rgb_frame: 'np.ndarray') -> Tuple[List[Tuple[int,int,int,int]], List['np.ndarray']]:
        locations = face_recognition.face_locations(rgb_frame, model=self._model)
        encodings = face_recognition.face_encodings(rgb_frame, locations)
        return locations, encodings


class FaceIdentifier:
    def __init__(self, known_dir: str, tolerance: float) -> None:
        self._tolerance = tolerance
        self._known_encodings: List['np.ndarray'] = []
        self._known_ids: List[str] = []
        self._tracked_encodings: List['np.ndarray'] = []
        self._tracked_ids: List[str] = []
        self._next_id = 1
        self._load_known(known_dir)

    def _load_known(self, directory: str) -> None:
        for fname in os.listdir(directory):
            path = os.path.join(directory, fname)
            image = face_recognition.load_image_file(path)
            encs = face_recognition.face_encodings(image)
            if encs:
                self._known_encodings.append(encs[0])
                self._known_ids.append(os.path.splitext(fname)[0])

    def identify(self, encoding: 'np.ndarray') -> str:
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


class GazeDetector:
    def __init__(self, threshold_px: int, duration: float) -> None:
        self._threshold = threshold_px
        self._duration = duration
        self._start_times: Dict[str, float] = {}
        self._state: Dict[str, bool] = {}

    def check(self, frame: 'np.ndarray', loc: Tuple[int,int,int,int], id_str: str, now: float) -> Optional[bool]:
        top, right, bottom, left = loc
        cx = (left + right) / 2
        w = frame.shape[1]
        facing = abs(cx - w/2) < self._threshold
        if facing:
            if id_str not in self._start_times:
                self._start_times[id_str] = now
            if now - self._start_times[id_str] >= self._duration and not self._state.get(id_str, False):
                self._state[id_str] = True
                return True
        else:
            if self._state.get(id_str, False):
                self._state[id_str] = False
                return False
            self._start_times.pop(id_str, None)
        return None


class FaceEngagementNode(Node):
    def __init__(self) -> None:
        super().__init__('face_engagement_node')
        self._declare_params()
        self._bridge = CvBridge()
        self._detector = FaceDetector(self.declare_parameter('detection_model', 'hog').value)
        self._identifier = FaceIdentifier(
            known_dir=self.declare_parameter('known_faces_dir', 'known_faces').value,
            tolerance=self.declare_parameter('match_tolerance', 0.6).value
        )
        self._gazer = GazeDetector(
            threshold_px=self.declare_parameter('gaze_threshold_px', 50).value,
            duration=self.declare_parameter('gaze_duration', 2.0).value
        )
        topic = self.declare_parameter('image_topic', '/image').value
        self.create_subscription(Image, topic, self._on_image, qos_profile=10)
        self._face_pub = self.create_publisher(String, 'face_event', qos_profile=10)
        self._gaze_pub = self.create_publisher(String, 'gaze_event', qos_profile=10)
        self._last_seen: Dict[str, float] = {}

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/image'),
            ('known_faces_dir', 'known_faces'),
            ('detection_model', 'hog'),
            ('match_tolerance', 0.6),
            ('gaze_threshold_px', 50),
            ('gaze_duration', 2.0)
        ]:
            self.declare_parameter(name, default)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        locs, encs = self._detector.detect(rgb)
        now = time.time()
        current_ids: List[str] = []
        for loc, enc in zip(locs, encs):
            id_str = self._identifier.identify(enc)
            current_ids.append(id_str)
            self._last_seen[id_str] = now
            # Publish face detected event with log
            face_msg = String(data=f'{id_str}:DETECTED')
            self.get_logger().info(f'Publishing face_event: {face_msg.data}')
            self._face_pub.publish(face_msg)
            # Check and publish gaze event with log
            gaze = self._gazer.check(frame, loc, id_str, now)
            if gaze is True:
                gaze_msg = String(data=f'{id_str}:ENGAGED')
                self.get_logger().info(f'Publishing gaze_event: {gaze_msg.data}')
                self._gaze_pub.publish(gaze_msg)
            elif gaze is False:
                gaze_msg = String(data=f'{id_str}:DISENGAGED')
                self.get_logger().info(f'Publishing gaze_event: {gaze_msg.data}')
                self._gaze_pub.publish(gaze_msg)
        self._cleanup(current_ids, now)

    def _cleanup(self, current_ids: List[str], now: float) -> None:
        timeout = 1.0
        for old_id, ts in list(self._last_seen.items()):
            if old_id not in current_ids and now - ts > timeout:
                # Publish lost face event with log
                lost_msg = String(data=f'{old_id}:LOST')
                self.get_logger().info(f'Publishing face_event: {lost_msg.data}')
                self._face_pub.publish(lost_msg)
                # Publish gaze disengage if needed
                if self._gazer._state.get(old_id, False):
                    disengage_msg = String(data=f'{old_id}:DISENGAGED')
                    self.get_logger().info(f'Publishing gaze_event: {disengage_msg.data}')
                    self._gaze_pub.publish(disengage_msg)
                self._last_seen.pop(old_id)
                self._gazer._state.pop(old_id, None)
                self._gazer._start_times.pop(old_id, None)

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down node')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceEngagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

