import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

L1 = 0.4
L2 = 0.4


def ik(rx, ry):
    """Two-link inverse kinematics for a target relative to the arm base."""
    d2 = rx * rx + ry * ry
    c2 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    q2 = math.acos(c2)
    q1 = math.atan2(ry, rx) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.declare_parameter('robot_id', 'r1')
        self.declare_parameter('base_x', -0.6)
        self.declare_parameter('speed', 0.3)
        self.rid = self.get_parameter('robot_id').value
        self.base_x = float(self.get_parameter('base_x').value)
        self.speed = float(self.get_parameter('speed').value)

        self.x = self.base_x
        self.y = 0.4
        self.queue = []
        self.halted = False
        self.dt = 0.02

        self.create_subscription(String, f'/{self.rid}/cmd', self.on_cmd, 10)
        self.state_pub = self.create_publisher(String, f'/{self.rid}/state', 10)
        self.joint_pub = self.create_publisher(JointState, f'/{self.rid}/joint_states', 10)
        self.create_timer(self.dt, self.step)
        self.create_timer(0.05, self.publish_state)
        self.get_logger().info(f'{self.rid} ready, base_x={self.base_x}')

    def on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('bad command JSON')
            return
        action = cmd.get('action')
        if action == 'move_to':
            tx, ty = float(cmd['x']), float(cmd['y'])
            if math.hypot(tx - self.base_x, ty) > L1 + L2 - 1e-6:
                self.get_logger().warn(f'target ({tx}, {ty}) out of reach')
                return
            self.queue.append((tx, ty))
        elif action == 'stop':
            self.halted = True
        elif action == 'go':
            self.halted = False
        elif action == 'clear':
            self.queue.clear()

    def step(self):
        if self.halted or not self.queue:
            return
        tx, ty = self.queue[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        move = self.speed * self.dt
        if dist <= move:
            self.x, self.y = tx, ty
            self.queue.pop(0)
        else:
            self.x += dx / dist * move
            self.y += dy / dist * move

    def publish_state(self):
        m = String()
        m.data = json.dumps({
            'id': self.rid, 'x': round(self.x, 4), 'y': round(self.y, 4),
            'busy': bool(self.queue), 'halted': self.halted, 'stamp': time.time(),
        })
        self.state_pub.publish(m)
        q1, q2 = ik(self.x - self.base_x, self.y)
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()
        j.name = [f'{self.rid}_joint1', f'{self.rid}_joint2']
        j.position = [q1, q2]
        self.joint_pub.publish(j)


def main():
    rclpy.init()
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
