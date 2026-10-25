import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ZoneMonitor(Node):
    def __init__(self):
        super().__init__('zone_monitor')
        self.declare_parameter('zone_x', 0.0)
        self.declare_parameter('zone_y', 0.0)
        self.declare_parameter('zone_radius', 0.12)
        self.declare_parameter('close_dist', 0.10)
        self.zx = float(self.get_parameter('zone_x').value)
        self.zy = float(self.get_parameter('zone_y').value)
        self.zr = float(self.get_parameter('zone_radius').value)
        self.close = float(self.get_parameter('close_dist').value)

        self.pos = {}
        for rid in ('r1', 'r2'):
            self.create_subscription(
                String, f'/{rid}/state', lambda m, r=rid: self.on_state(r, m), 10)
        self.pub = self.create_publisher(String, '/zone/status', 10)

        self.zone_events = 0
        self.close_events = 0
        self.zone_time = 0.0
        self.in_conflict = False
        self.in_close = False
        self.create_timer(0.02, self.check)
        self.create_timer(0.5, self.report)

    def on_state(self, rid, msg):
        d = json.loads(msg.data)
        self.pos[rid] = (d['x'], d['y'])

    def in_zone(self, p):
        return math.hypot(p[0] - self.zx, p[1] - self.zy) <= self.zr

    def check(self):
        if len(self.pos) < 2:
            return
        p1, p2 = self.pos['r1'], self.pos['r2']
        conflict = self.in_zone(p1) and self.in_zone(p2)
        close = math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < self.close
        if conflict:
            self.zone_time += 0.02
        if conflict and not self.in_conflict:
            self.zone_events += 1
            self.get_logger().warn('ZONE CONFLICT: both arms in the shared zone')
        if close and not self.in_close:
            self.close_events += 1
            self.get_logger().warn('CLOSE APPROACH between end effectors')
        self.in_conflict = conflict
        self.in_close = close

    def report(self):
        m = String()
        m.data = json.dumps({
            'zone_events': self.zone_events,
            'close_events': self.close_events,
            'zone_conflict_seconds': round(self.zone_time, 2),
        })
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ZoneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
