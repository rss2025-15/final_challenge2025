import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from vs_msgs.msg import ConeLocation, ConeLocationPixel
import numpy as np
import math
# from final_challenge_stata.detector import Detector
from std_msgs.msg import Int32, Bool
from .detector import Detector
# from PIL import Image, ImageDraw, ImageFont

class TrafficNode(Node):
    def __init__(self):
        super().__init__("traffic_controller")

        self.drive_topic = "/vesc/low_level/input/navigation"
        self.lookahead = 2.0  # FILL IN #
        self.speed = 0.5  # FILL IN #
        self.wheelbase_length = .46  # FILL IN #
        self.max_steer = 0.78
        self.odom_topic =  "/odom"
        self.target_x = None
        self.target_y = None
        self.start_time = None

        # subscribers
        # self.homography_subscriber = self.create_subscription(ConeLocation, '/relative_cone', self.homography_callback, 1)
        self.camera_subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.camera_callback, 1)
        # self.start_detection_subscriber = self.create_subscription(Bool, 'start_detection',self.start_detection_callback,  1)

        # publishers
        # self.command_pub = self.create_publisher(Bool, "/traffic_cmd", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.bridge = CvBridge()


        self.color_thresh = {
            'red': [{'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)}],
            'yellow': [{'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)}],
            'green': [{'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)}],
            }

        self.color_idx = {0: 'red', 1: 'yellow', 2: 'green'}

        self.get_logger().info("Traffic Node started")


    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().info(f"Failed to convert image {e}")

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        areas = {'red': 0, 'yellow': 0, 'green': 0}

        for color, ranges in self.color_thresh.items():
            mask = None
            for rng in ranges:
                m = cv2.inRange(hsv_img, rng['lower'], rng['upper'])
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            for cnt in contours:
                max_area = max(max_area, cv2.contourArea(cnt))
            areas[color] = max_area

        signal = max(areas, key=areas.get)
        should_stop = (signal == 'red')

        mask = None
        for rng in self.color_thresh[signal]:
            m = cv2.inRange(hsv_img, rng['lower'], rng['upper'])
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)
        

        # publish stop
        # stop_msg = Bool()
        # stop_msg.data = should_stop
        # self.command_pub.publish(stop_msg)
        
        # TODO: REPLACE THIS WITH SAFETY CONTROLLER LOGIC
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = 0.0
        self.drive_pub.publish(drive_cmd)
        self.get_logger().info(f'Detected {signal}, should_stop={should_stop}')
        self.get_logger().info(f'{areas}')

        
def main(args=None):
    rclpy.init(args=args)
    traffic = TrafficNode()
    rclpy.spin(traffic)
    rclpy.shutdown()

if __name__=="__main__":
    main()

        # for i, color in enumerate(['red', 'yellow', 'green']):
        #     mask = cv2.inRange(hsv_img, color_thresh[color]['lower'], color_thresh[color]['upper'])
        #     mask.save
        #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #     #bounding_box = ((0, 0), (0, 0))  # Default if no object found
        #     largest_area = 0
        #     best_contour = None

        #     for cnt in contours:
        #         area = cv2.contourArea(cnt)
        #         if area > largest_area:
        #             largest_area = area
        #             best_contour = cnt

        #     if best_contour is not None:
        #         color_area[i]=largest_area

        # signal = color_idx[color_area.index(max(color_area))]
        # print(signal)

