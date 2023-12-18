import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import ament_index_python

from autorace_communication_gazebo_ogry.action import *

import cv2
import numpy as np
import os

THRESHOLD = 10

order = [
    "intersection_sign",
    "parking_sign",
    "crossing_sign",
    "tunnel_sign"
]

current = 0

def open_image(sift, path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"{path} not found")
    kp, des = sift.detectAndCompute(img, None)
    des = np.float32(des)
    print(path, des, img)
    return kp, des

def fnCalcMSE(arr1, arr2):
    squared_diff = (arr1 - arr2) ** 2
    sum = np.sum(squared_diff)
    num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
    err = sum / num_all
    return err

MIN_MSE_DECISION = 50000


class SignDetector(Node):
    def __init__(self):
        super().__init__("sign_detector")
        self.img_subscription = self.create_subscription(Image, "/color/image", self.detect, 10)
        self.driver_state = self.create_publisher(Bool, "/driver_state", 10)
        self.turn_direction = self.create_publisher(String, '/intersection/turn_dir', 10)

        # autorace_communication_gazebo_ogry.action
        self.action_servers = {
            "intersection_sign": (ActionClient(self, Intersection, "intersection"), Intersection.Goal(direction = "none")),
            # "construction_sign": (ActionClient(self, Obstacles, "construction"), Obstacles.Goal()),
            "parking_sign": (ActionClient(self, Parking, "parking"), Parking.Goal()),
            "crossing_sign": (ActionClient(self, Crosswalk, "crosswalk"), Crosswalk.Goal()),
            "tunnel_sign": (ActionClient(self, Tunnel, "tunnel"), Tunnel.Goal())
        }

        for key, (client, _) in self.action_servers.items():
            self.get_logger().info('Waiting {0} server'.format(key))
            client.wait_for_server()

        self.get_logger().info("ALL ACTION SERVERS STARTED")
        self.cvBridge = CvBridge()

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.sift = cv2.SIFT_create()

        driver_path = os.path.join(ament_index_python.get_package_share_directory("autorace_vision_gazebo_ogry"), "images")

        self.during_action = [
            ('traffic_left', open_image(self.sift, os.path.join(driver_path, "turn_left.png")), MIN_MSE_DECISION),
            ('traffic_right', open_image(self.sift, os.path.join(driver_path, "turn_right.png")), MIN_MSE_DECISION),
        ]

        self.signs = [
            ('intersection_sign' , open_image(self.sift, os.path.join(driver_path, "intersection.png")), MIN_MSE_DECISION),
            # ('construction_sign' , open_image(self.sift, os.path.join(driver_path, "construction.png")), 250000),
            ('parking_sign' , open_image(self.sift, os.path.join(driver_path, "parking.png")), MIN_MSE_DECISION),
            ('crossing_sign' , open_image(self.sift, os.path.join(driver_path, "crosswalk.png")), MIN_MSE_DECISION),
            ('tunnel_sign' , open_image(self.sift, os.path.join(driver_path, "tunnel.png")), MIN_MSE_DECISION),
            
            # ('traffic_red', open_image(self.sift, os.path.join(traffic_light_path, "traffic_light_red.png"))),
            # ('traffic_green', open_image(self.sift, os.path.join(traffic_light_path, "traffic_green.png")))
        ]
        print("initialized")
        self.robot_started = False
        self.action_processing = False
        self.counter = 0
        self.current = 0


    
    def getDetected(self, des, img_list, thres = 20):

        for name, (kp_, des_), min_mse in img_list:
            matches = self.flann.knnMatch(des, des_, k=2)
            good_intersection = []
            for m,n in matches:
                if m.distance < 0.55*n.distance:
                    good_intersection.append(m)
            if len(good_intersection)>=thres:
                return name, len(good_intersection)
        return None, None

    def detect(self, msg):
        self.counter += 1
        img = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        
        if not self.robot_started:
            if np.any(np.logical_and(img[:, :, 1] > 200, np.logical_and(img[:, :, 0] < 200, img[:, :, 2] < 200))):
                self.robot_started = True
                self.get_logger().info("GREEN LIGHT")
                msg = Bool()
                msg.data = True
                self.driver_state.publish(msg)
            return
        
        kp, des = self.sift.detectAndCompute(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)[:280], None)


        if self.action_processing:
            kp, des = self.sift.detectAndCompute(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None)
            name, count = self.getDetected(des, self.during_action, 15)
            if name is None:
                return
            msg = String()
            if name == "traffic_left":
                msg.data = "left"
            elif name == "traffic_right":
                msg.data = "right"
            self.turn_direction.publish(msg)
            self.get_logger().info(f"{name} {count}")
            return

        
        name, count = self.getDetected(des, self.signs)
        
        
        if name in self.action_servers.keys():
            if order[self.current] != name:
                return
            self.current += 1
            self.get_logger().info(f"{name} {count}")
            (client, cls) = self.action_servers[name]
            client.wait_for_server()
            goal = cls
            self.action_processing = True
            self.future = client.send_goal_async(goal)
            self.future.add_done_callback(self.goal_response_callback)
            return
        
        
        
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_processing = False
        self.get_logger().info("action has been executed correctly")



def main(args=None):
    rclpy.init(args=args)

    sign_detector = SignDetector()

    rclpy.spin(sign_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sign_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()