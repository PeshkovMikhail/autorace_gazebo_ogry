import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np

from autorace_communication_gazebo_ogry.action import Intersection
import time

def fnCalcMSE(arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        sum = np.sum(squared_diff)
        num_all = arr1.shape[0] #cv_image_input and 2 should have same shape
        err = sum / num_all
        return err

class IntersectionMission(Node):
    def __init__(self):
        super().__init__('intersection_action_server')
        self.turn_dir_subscription = self.create_subscription(String, "/intersection/turn_dir", self.set_turn_dir, 10)
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.update_odom, 10)

        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.driver_state = self.create_publisher(Bool, "/driver_state", 10)

        self.turn_dir = None
        self.z_angle = 0
        self.pose = None

        self.action_server = ActionServer(self, Intersection, "intersection", self.execute_callback)

        

    def execute_callback(self, goal_handle: Intersection.Goal):
        self.get_logger().info("task received")
        self.loop_rate = self.create_rate(50)
        while fnCalcMSE(self.pose, np.array([0.70, 0.98])) > 0.01:
            self.loop_rate.sleep()
        driver_state_msg = Bool()
        driver_state_msg.data = False
        self.driver_state.publish(driver_state_msg)
        msg = Twist()
        self.cmd_vel.publish(msg)
        self.get_logger().info(f"Driver disabled {self.pose}")
        
        goal_handle.succeed()
        result = Intersection.Result()
        result.finished = True
        return result

    def set_turn_dir(self, msg):
        if self.turn_dir is None:
            self.turn_dir = msg.data

    def update_odom(self, msg: Odometry):
        self.get_logger().info("odom updated")
        q = msg.pose.pose.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.z_angle = np.arctan2(t3, t4)
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

def main(args=None):
    rclpy.init(args=args)

    intersection_action_server = IntersectionMission()

    rclpy.spin(intersection_action_server)


if __name__ == '__main__':
    main()