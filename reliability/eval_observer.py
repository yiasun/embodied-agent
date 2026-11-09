"""Read-only /map, /plan, /odom and optional BT log capture; never controls ROS."""
import math
import time

from .observer import NavigationObserver
from .safety import ObstacleError
from scenes.runtime_guard import check_map_message


class EvaluationObserver(NavigationObserver):
    def __init__(self, bundle):
        super().__init__(bundle)
        from nav_msgs.msg import Odometry
        from nav2_msgs.msg import BehaviorTreeLog
        self.bt = []; self.odom = None; self.previous_path = None; self.checked_map = None; self.sim_now = None
        self.node.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', (m, time.monotonic())), 10)
        self.node.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.on_bt, 50)

    def on_bt(self, message):
        for event in message.event_log:
            self.bt.append({'node_name': event.node_name, 'uid': event.uid,
                            'previous_status': event.previous_status, 'current_status': event.current_status,
                            'internal_stamp': event.timestamp.sec + event.timestamp.nanosec/1e9,
                            'received_mono': time.monotonic(),
                            'received_sim': self.node.get_clock().now().nanoseconds/1e9})
        if len(self.bt) > 5000:
            raise ObstacleError('BT_EVENT_BUFFER_OVERFLOW')

    def poll(self, metrics, task):
        self.rclpy.spin_once(self.node, timeout_sec=0.005)
        if self.map is not None and self.map is not self.checked_map:
            check_map_message(self.map, self.bundle)
            self.checked_map = self.map
        metrics.bt_events.extend(self.bt); self.bt.clear()
        sim_now = self.node.get_clock().now().nanoseconds/1e9
        self.sim_now = sim_now
        if self.path is not None and self.path is not self.previous_path:
            self.previous_path = self.path
            message, received = self.path
            if message.header.frame_id != 'map':
                raise ObstacleError('EVALUATION_PATH_FRAME_MISMATCH')
            if task is not None and time.monotonic()-received <= 2:
                points = [[p.pose.position.x, p.pose.position.y] for p in message.poses]
                if points and math.dist(points[-1], [task['x'], task['y']]) <= 0.6:
                    stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
                    metrics.path(points, stamp, sim_now, task['id'])
        if self.odom:
            odom, received = self.odom; stamp = odom.header.stamp.sec + odom.header.stamp.nanosec/1e9
            if (odom.header.frame_id == 'odom' and odom.child_frame_id in ('base_footprint', 'base_link')
                    and time.monotonic()-received < 0.5 and stamp > 0 and -0.1 <= sim_now-stamp <= 0.5):
                velocity = [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z]
                if all(math.isfinite(v) for v in velocity):
                    return {'stamp_sim': stamp, 'frame': odom.child_frame_id, 'velocity_body': velocity}
        return None
