import time
from typing import Dict, Set

from rclpy.node import Node
from std_msgs.msg import String


class EngagementManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('engagement_manager_node')
        self._declare_params()
        
        self.create_subscription(String, 'gaze_status', self._on_gaze_status, qos_profile=10)
        self._face_event_pub = self.create_publisher(String, 'face_event', qos_profile=10)
        self._gaze_event_pub = self.create_publisher(String, 'gaze_event', qos_profile=10)
        
        self.get_logger().info('Engagement Manager Node started - Input topic: /gaze_status')
        self.get_logger().info('Engagement Manager Node - Output topics: /face_event, /gaze_event')
        
        self._active_faces: Set[str] = set()
        self._last_seen: Dict[str, float] = {}
        self._engagement_state: Dict[str, bool] = {}
        self._face_timeout = self.get_parameter('face_timeout').value
        
        self.create_timer(0.1, self._check_face_timeout)

    def _declare_params(self) -> None:
        for name, default in [
            ('face_timeout', 1.0)
        ]:
            self.declare_parameter(name, default)

    def _on_gaze_status(self, msg: String) -> None:
        data = msg.data.split('|')
        if len(data) < 2:
            return
            
        face_id = data[0]
        status = data[1]
        now = time.time()
        
        if face_id not in self._active_faces:
            self._active_faces.add(face_id)
            face_msg = String(data=f'{face_id}:DETECTED')
            self.get_logger().info(f'Publishing face_event: {face_msg.data}')
            self._face_event_pub.publish(face_msg)
        
        self._last_seen[face_id] = now
        
        if status == 'ENGAGED' and not self._engagement_state.get(face_id, False):
            self._engagement_state[face_id] = True
            gaze_msg = String(data=f'{face_id}:ENGAGED')
            self.get_logger().info(f'Publishing gaze_event: {gaze_msg.data}')
            self._gaze_event_pub.publish(gaze_msg)
        elif status == 'DISENGAGED' and self._engagement_state.get(face_id, False):
            self._engagement_state[face_id] = False
            gaze_msg = String(data=f'{face_id}:DISENGAGED')
            self.get_logger().info(f'Publishing gaze_event: {gaze_msg.data}')
            self._gaze_event_pub.publish(gaze_msg)

    def _check_face_timeout(self) -> None:
        now = time.time()
        lost_faces = []
        
        for face_id in list(self._active_faces):
            if face_id in self._last_seen and now - self._last_seen[face_id] > self._face_timeout:
                lost_faces.append(face_id)
        
        for face_id in lost_faces:
            self._active_faces.discard(face_id)
            face_msg = String(data=f'{face_id}:LOST')
            self.get_logger().info(f'Publishing face_event: {face_msg.data}')
            self._face_event_pub.publish(face_msg)
            
            if self._engagement_state.get(face_id, False):
                self._engagement_state[face_id] = False
                gaze_msg = String(data=f'{face_id}:DISENGAGED')
                self.get_logger().info(f'Publishing gaze_event: {gaze_msg.data}')
                self._gaze_event_pub.publish(gaze_msg)
            
            self._last_seen.pop(face_id, None)
            self._engagement_state.pop(face_id, None)

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down engagement manager node')
        super().destroy_node()


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = EngagementManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()