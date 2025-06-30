import time
from typing import Dict, Optional

from rclpy.node import Node
from std_msgs.msg import String


class GazeDetector:
    def __init__(self, threshold_px: int, duration: float) -> None:
        self._threshold = threshold_px
        self._duration = duration
        self._start_times: Dict[str, float] = {}
        self._state: Dict[str, bool] = {}

    def check(self, center_x: float, frame_width: float, face_id: str, now: float) -> Optional[bool]:
        facing = abs(center_x - frame_width/2) < self._threshold
        if facing:
            if face_id not in self._start_times:
                self._start_times[face_id] = now
            if now - self._start_times[face_id] >= self._duration and not self._state.get(face_id, False):
                self._state[face_id] = True
                return True
        else:
            if self._state.get(face_id, False):
                self._state[face_id] = False
                return False
            self._start_times.pop(face_id, None)
        return None

    def reset_face(self, face_id: str) -> bool:
        was_engaged = self._state.get(face_id, False)
        self._state.pop(face_id, None)
        self._start_times.pop(face_id, None)
        return was_engaged


class GazeAnalysisNode(Node):
    def __init__(self) -> None:
        super().__init__('gaze_analysis_node')
        self._declare_params()
        self._gazer = GazeDetector(
            threshold_px=self.get_parameter('gaze_threshold_px').value,
            duration=self.get_parameter('gaze_duration').value
        )
        
        self.create_subscription(String, 'face_identities', self._on_face_identity, qos_profile=10)
        self._gaze_status_pub = self.create_publisher(String, 'gaze_status', qos_profile=10)
        
        self.get_logger().info('Gaze Analysis Node started - Input topic: /face_identities')
        self.get_logger().info('Gaze Analysis Node - Output topic: /gaze_status')

    def _declare_params(self) -> None:
        for name, default in [
            ('gaze_threshold_px', 50),
            ('gaze_duration', 2.0)
        ]:
            self.declare_parameter(name, default)

    def _on_face_identity(self, msg: String) -> None:
        data = msg.data.split('|')
        if len(data) < 7:
            return
            
        face_id = data[0]
        center_x = float(data[1])
        center_y = data[2]
        width = data[3]
        height = data[4]
        frame_width = float(data[5])
        frame_height = data[6]
        
        now = time.time()
        gaze_result = self._gazer.check(center_x, frame_width, face_id, now)
        
        if gaze_result is True:
            gaze_msg = String(data=f"{face_id}|ENGAGED|{center_x}|{center_y}|{width}|{height}")
            self._gaze_status_pub.publish(gaze_msg)
        elif gaze_result is False:
            gaze_msg = String(data=f"{face_id}|DISENGAGED|{center_x}|{center_y}|{width}|{height}")
            self._gaze_status_pub.publish(gaze_msg)
        else:
            gaze_msg = String(data=f"{face_id}|TRACKING|{center_x}|{center_y}|{width}|{height}")
            self._gaze_status_pub.publish(gaze_msg)


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = GazeAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()