#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import cv2

from cv_bridge import CvBridge, CvBridgeError
from vs_msgs.msg import ConeLocationPixel

from sensor_msgs.msg import Image

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.declare_parameter("white_lower_lims", [0, 0, 0])
        self.declare_parameter("white_upper_lims", [0, 0, 0])
        self.declare_parameter("focus_area", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("line_angle_limit", 0.0)

        self.white_lower_lims = self.get_parameter("white_lower_lims").value
        self.white_upper_lims = self.get_parameter("white_upper_lims").value
        self.focus_area = self.get_parameter("focus_area").value
        self.line_angle_limit = self.get_parameter("line_angle_limit").get_parameter_value().double_value

        self.get_logger().info(f'WHITE PIXEL HSV LIMITS: {self.white_lower_lims, self.white_upper_lims}')
        
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cv_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "camera_topic", self.image_callback, 5)
        self.bridge = CvBridge()

    def image_callback(self, img_msg):
        # get image and convert to HSV
        image_cv = self.bridge.imgmsg_to_cv2(img_msg, "bgra8")
        hsv_img = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        # cut the image to the selected area
        min_x, min_y, max_x, max_y = self.focus_area
        min_x, max_x = int(min_x*hsv_img.shape[0]), int(max_x*hsv_img.shape[0])
        min_y, max_y = int(min_y*hsv_img.shape[1]), int(max_y*hsv_img.shape[1])
        focused_img = hsv_img[min_x:max_x, min_y:max_y]

        # filter white pixels
        lower_hsv = np.array(self.white_lower_lims, dtype=np.uint8)
        upper_hsv = np.array(self.white_upper_lims, dtype=np.uint8)
        
        mask = cv2.inRange(focused_img, lower_hsv, upper_hsv)

        # dilate & erode
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        # mask = cv2.dilate(mask, kernel, iterations=1)

        # Hough transform
        lines = cv2.HoughLines(mask, 1, np.pi / 180, 150, None, 0, 0)
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        left_line, right_line = None, None # (r, theta, slope, offset)
        # theta also gives us which lanes are to the left and which are to the right, to know which lane we are in
        if lines is not None:
            for i in range(len(lines)):
                r = lines[i][0][0]
                theta = lines[i][0][1]
                slope = -math.cos(theta)/math.sin(theta)
                offset = r/math.sin(theta)
                if theta < 0:
                    theta += np.pi
                if theta > np.pi:
                    theta -= np.pi
                if theta > np.pi/2:
                    theta -= np.pi
                # self.get_logger().info(f'LINE {i} WITH POLAR PARAMS: {r, theta}')
                # discard if abs(theta) too large -> perpendicular to the track lines
                if abs(theta) > self.line_angle_limit/180*np.pi:
                    continue
                # self.get_logger().info(f'LINE {i} VALID')
                mask_rgb = cv2.line(mask_rgb, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (0.0, 0.0, 255.0), 5)
                if theta < 0 and (left_line is None or abs(left_line[1]) > abs(theta)):
                    left_line = (r, theta, slope, offset)
                if theta > 0 and (right_line is None or abs(right_line[1]) > abs(theta)):
                    right_line = (r, theta, slope, offset)

            if left_line is not None:
                r, theta, slope, offset = left_line
                mask_rgb = cv2.line(mask_rgb, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (255.0, 0.0, 0.0), 5)
            if right_line is not None:
                r, theta, slope, offset = right_line
                mask_rgb = cv2.line(mask_rgb, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (0.0, 255.0, 0.0), 5)

            x_pixel, y_pixel = 0, 0
            cone_pixel = ConeLocationPixel()
            cone_pixel.u = float(x_pixel+min_x)
            cone_pixel.v = float(y_pixel+min_y)
            self.cone_pub.publish(cone_pixel)

        # debug_msg = self.bridge.cv2_to_imgmsg(mask, "8UC1")
        debug_msg = self.bridge.cv2_to_imgmsg(mask_rgb, "rgb8")
        # debug_msg = self.bridge.cv2_to_imgmsg(focused_img, "bgr8")

        # cv2.imshow("image", mask_rgb)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        self.debug_pub.publish(debug_msg)   

def main():
    rclpy.init()
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()