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
from final_challenge_stata.detector import Detector
from std_msgs.msg import Int32, Bool
from .detector import Detector
from PIL import Image, ImageDraw, ImageFont

class TrafficNode(Node):
    def __init__(self):
        super().__init__("traffic")

        self.drive_topic = "/vesc/low_level/input/navigation"
        self.lookahead = 2.0  # FILL IN #
        self.speed = 0.5  # FILL IN #
        self.wheelbase_length = .46  # FILL IN #
        self.max_steer = 0.78
        self.odom_topic =  "/odom"
        self.target_x = None
        self.target_y = None
        self.start_time = None

        self.bridge = CvBridge()
        color_thresh = {'red': {'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)},
                'yellow': {'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)},
                'green': {'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)},
                }

        color_idx = {0: 'red', 1: 'yellow', 2: 'green'}

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for i, color in enumerate(['red', 'yellow', 'green']):
            mask = cv2.inRange(hsv_img, color_thresh[color]['lower'], color_thresh[color]['upper'])
            mask.save
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            #bounding_box = ((0, 0), (0, 0))  # Default if no object found
            largest_area = 0
            best_contour = None

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > largest_area:
                    largest_area = area
                    best_contour = cnt

            if best_contour is not None:
                color_area[i]=largest_area

        signal = color_idx[color_area.index(max(color_area))]
        print(signal)

