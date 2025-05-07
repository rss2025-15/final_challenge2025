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
from .traffic_light import color_detect
# from PIL import Image, ImageDraw, ImageFont

class TrafficNode(Node):
    def __init__(self):
        super().__init__("traffic_controller")

        self.drive_topic = "/vesc/low_level/input/navigation"
        self.lookahead = 2.0  # FILL IN #
        self.speed = 0.75  # FILL IN #
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
        self.stop_pub = self.create_publisher(Bool, "/traffic_stop", 1)
        self.bridge = CvBridge()

        # detector
        self.detector = Detector()
        self.detector.set_threshold(0.7)

        self.cached_bounding_box = None
        self.signal = None

        self.color_idx = {0: 'red', 1: 'yellow', 2: 'green'}

        self.get_logger().info("Traffic Node started")


    def camera_callback(self, camera_msg):
        # self.get_logger().info("calling camera")
        image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        image_debug = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        results = self.detector.predict(image)
        predictions = results["predictions"]
        for prediction in predictions:
            if prediction[1] == 'traffic light':
                x1, y1, x2, y2 = prediction[0] 
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                cropped_img = image[y1:y2+1, x1:x2+1]
			    
                self.signal = self.color_detect(cropped_img)

                cv2.rectangle(image_debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image_debug, f"detected color: {self.signal}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                debug_msg = self.bridge.cv2_to_imgmsg(image_debug, "bgr8")
                
                self.debug_pub.publish(debug_msg)

                stop_cmd = (self.signal == 'red')
                self.get_logger().info(f'Detected {self.signal}, should_stop={stop_cmd}')

                # mask = None
                # for rng in self.color_thresh[signal]:
                #     m = cv2.inRange(hsv_img, rng['lower'], rng['upper'])
                #     mask = m if mask is None else cv2.bitwise_or(mask, m)
                # image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

                # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                # self.debug_pub.publish(debug_msg)

                if stop_cmd:
                    # drive_cmd = AckermannDriveStamped()
                    # drive_cmd.drive.speed = 0.0
                    # self.drive_pub.publish(drive_cmd)
                    self.cached_bounding_box = (x1, y1, x2, y2)
                    stop_cmd_msg = Bool()
                    stop_cmd_msg.data = True
                    self.stop_pub.publish(stop_cmd_msg)
                    return
                else:
                    self.cached_bounding_box = None
                # else:
                #     stop_cmd_msg.data = False
                #     self.stop_pub.publish(stop_cmd_msg)
                    # self.drive_cmd()
            # else:
            #     self.drive_cmd()

        if self.cached_bounding_box:
            self.get_logger().info("bounding box cached")
            x1, y1, x2, y2 = self.cached_bounding_box
            cv2.rectangle(image_debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image_debug, f"detected color: {self.signal}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            debug_msg = self.bridge.cv2_to_imgmsg(image_debug, "bgr8")
            self.debug_pub.publish(debug_msg)
            
            cropped_img = image[y1:y2+1, x1:x2+1]
            
            signal = self.color_detect(cropped_img)
            stop_cmd = (signal == 'red')
            self.get_logger().info(f'Detected {signal}, should_stop={stop_cmd}')

            # mask = None
            # for rng in self.color_thresh[signal]:
            #     m = cv2.inRange(hsv_img, rng['lower'], rng['upper'])
            #     mask = m if mask is None else cv2.bitwise_or(mask, m)
            # image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            # self.debug_pub.publish(debug_msg)

            if stop_cmd:
                # drive_cmd = AckermannDriveStamped()
                # drive_cmd.drive.speed = 0.0
                # self.drive_pub.publish(drive_cmd)
                self.cached_bounding_box = (x1, y1, x2, y2)
                stop_cmd_msg = Bool()
                stop_cmd_msg.data = True
                self.stop_pub.publish(stop_cmd_msg)
                return
            else:
                self.cached_bounding_box = None
            # else:
            #     stop_cmd_msg.data = False
            #     self.stop_pub.publish(stop_cmd_msg)
                # self.drive_cmd()
        # else:
        #     self.drive_cmd()

        stop_cmd = Bool()
        stop_cmd.data = False
        self.stop_pub.publish(stop_cmd)

    # def drive_cmd(self):
    #     # self.get_logger().info("sending drive commands")
    #     drive_msg = AckermannDriveStamped()
    #     drive_msg.drive.speed = self.speed
    #     self.drive_pub.publish(drive_msg)

    def color_detect(self, img):
        # my fun constants
        color_thresh = {'red': {'lower': (160, 180, 180), 'upper': (179,255,255)},
                        'yellow': {'lower': np.array([20, 180, 180], dtype=np.uint8), 'upper': np.array([35,255,255], dtype=np.uint8)},
                        'green': {'lower': np.array([75, 180, 180], dtype=np.uint8), 'upper': np.array([100,255,255], dtype=np.uint8)},
                        }
        color_idx = {0: 'red', 1: 'yellow', 2: 'green'}
        # we get predictions out
        # go through them and get the traffic light one

        # if traffic light, then we do color segmentation by the three different ranges

        color_area = [-1,-1,-1]

        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        for i, color in enumerate(['red', 'yellow', 'green']):
            mask = cv2.inRange(hsv_img, color_thresh[color]['lower'], color_thresh[color]['upper'])
            cv2.imwrite('attempted_mask.png', mask)
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
        print(color_area)
        signal = color_idx[color_area.index(max(color_area))]
        return signal

        
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

