import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np

from autorace_communication_gazebo_ogry.action import Tunnel

class TunnelMission(Node):
    def __init__(self):
        super().__init__('tunnel_action_server')
        self.action_server = ActionServer(self, Tunnel, "tunnel", self.execute_callback)

        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.update_odom, 10)

        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.z_angle = 0
        self.pose = None

    def execute_callback(self, goal_handle):
        
        pass

    def update_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.z_angle = np.arctan2(t3, t4)
        self.pose = msg.pose.pose.position

def main(args=None):
    rclpy.init(args=args)

    tunnel_action_server = TunnelMission()

    rclpy.spin(tunnel_action_server)


if __name__ == '__main__':
    main()