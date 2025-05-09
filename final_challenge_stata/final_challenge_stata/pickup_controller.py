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
import os


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")

        ####
        self.drive_topic = "/vesc/low_level/input/navigation"
        self.lookahead = 2.0  # FILL IN #
        self.speed = 0.5  # FILL IN #
        self.wheelbase_length = .46  # FILL IN #
        self.max_steer = 0.78
        self.angle_thres = np.pi/6.0
        self.odom_topic =  "/odom"
        self.target_x = None
        self.target_y = None
        self.start_time = None
        self.min_turn_radius = self.wheelbase_length/math.tan(self.max_steer)

        self.bridge = CvBridge()

        ###Detector
        self.detector = Detector()
        self.detector.set_threshold(0.1)
        ####
        self.send_drive_command = True
        self.last_switch_command = True
        self.detect = False
        self.shrinkray_count = 1
        self.detection_valid_time = None
        self.detection_valid_thres = 1.5
        self.start_searching = False
        # switch
        # Bool, /switch_parking
        # /shrinkray_count
        # / Int32

        ####

        ###subscribers

        self.homography_subscriber = self.create_subscription(ConeLocation, '/relative_cone', self.homography_callback, 1)
        self.camera_subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.camera_callback, 1)
        self.start_detection_subscriber = self.create_subscription(Bool, 'start_detection',self.start_detection_callback,  1)
        self.start_searching_subscriber = self.create_subscription(Bool, 'start_searching',self.start_searching_callback,  1)
        self.react_timer = self.create_timer(1/30.0, self.react_callback)

        ###publishers
        self.pixel_publisher = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.switch_pub = self.create_publisher(Bool, '/switch_parking', 1)
        self.shrinkray_count_pub = self.create_publisher(Int32, '/shrinkray_count', 1)
        ###

        ####
        self.get_logger().info("Detector Initialized")

        self.backup_status = 0 # 0 not doing anything -1 backwards, 1 going forwards
        self.backup_status_start = None
        self.front_backup_increment_time = 0.5
        self.backup_increment_time = 1.5
        self.image = None

    def camera_callback(self, camera_msg):
        # extract the image out of camera msg
        image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        #pass this image to the detector for predictions
        results = self.detector.predict(image)
        predictions = results["predictions"]
        # self.get_logger().info('outside loop')
        #format
        #((356.390625, 110.8359375, 448.921875, 171.703125), 'banana')
        for prediction in predictions:
            # self.get_logger().info(f'predicion is {prediction}')

            if prediction[1] == 'banana':
                    # return if we are not supposed to be detecting right now
                if not self.detect:
                    self.get_logger().info('self.detect is turned off rn')
                    return
                #if last switch command was True only then publish message to switch
                self.get_logger().info('Sending Switch Command of True')

                #turn off the regular follower
                self.switch_cmd(True)

                # detector returns (xmin, ymin, xmax, ymax)
                x1_f, y1_f, x2_f, y2_f = prediction[0]

                x1, y1, x2, y2 = map(int, (x1_f, y1_f, x2_f, y2_f))
                self.get_logger().info(f'rectangle_box is {x1,x2,y1,y2}'  )

                middle_x_pixel = (x1 + x2) // 2
                bottom_y_pixel = y2
                #####debug
                # add box around orig image
                cv2.rectangle(image, (x1,y1), (x2, y2), (0,255,0), 2)
                cv2.putText(image, " ", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                # self.get_logger().info(f"{cone_pixel.u, cone_pixel.v}")
                self.debug_pub.publish(debug_msg)
                self.image = image

                ####debug

                middle_x_pixel = (x1+x2)//2
                bottom_y_pixel = y2
                self.get_logger().info(f'The middle x pixel: {middle_x_pixel}')
                self.get_logger().info(f'The bottom y pixel: {bottom_y_pixel}')
                cone_pixel = ConeLocationPixel()
                cone_pixel.u = float(middle_x_pixel)
                cone_pixel.v = float(bottom_y_pixel)
                self.pixel_publisher.publish(cone_pixel)

        #assume we finally have the x and y pixels

    def homography_callback(self, position_msg: ConeLocation ):
        if not self.detect:
            return
        self.target_x = position_msg.x_pos
        self.target_y = position_msg.y_pos
        self.get_logger().info(f'target x received as {self.target_x}')
        self.get_logger().info(f'target y received as {self.target_y}')
        self.detection_valid_time = self.get_clock().now().nanoseconds*float(10**-9)

    def react_callback(self):
        # self.get_logger().info(f'DETECT: {self.detect}')
        if not self.detect:
            return
        if self.start_searching and self.detection_valid_time is None:
            # turn in place
            self.switch_cmd(True)
            # self.drive_cmd(self.max_steer, -0.7)
            self.get_logger().info('SEARCHING FOR BANANA')
            steer_dir = 1
            self.drive_cmd(-steer_dir*self.max_steer*self.backup_status, self.backup_status*self.speed)
            backup_time = self.front_backup_increment_time if self.backup_status == 1 else self.backup_increment_time
            if self.backup_status == 0:
                self.backup_status = -1
                self.backup_status_start = self.get_clock().now().nanoseconds*float(10**-9)
            elif (self.get_clock().now().nanoseconds*float(10**-9) - self.backup_status_start) > backup_time:
                self.backup_status = -self.backup_status
                self.backup_status_start = self.get_clock().now().nanoseconds*float(10**-9)
            return
        elif self.detection_valid_time is None:
            return

        self.switch_cmd(True)

        curr_time = self.get_clock().now().nanoseconds*float(10**-9)

        #pose call back should continue working as it is right now

        #lets find distance between robot and target point
        # target_distance = math.sqrt((self.target_x)**2 + (self.target_y)**2)
        # self.get_logger().info(f'distance of robot from target point is {target_distance}')

        # angle_in_robot_frame = math.atan2(self.target_y, self.target_x)
        # self.get_logger().info(f'angle in robot frame is  is {angle_in_robot_frame}')

        # turn_radius = target_distance / (2*math.sin(angle_in_robot_frame))
        # steer_angle = math.atan(self.wheelbase_length/turn_radius)
        self.cmd_speed = 0.7

        self.relative_x = self.target_x
        self.relative_y = self.target_y

        lookahead = np.linalg.norm([self.relative_x, self.relative_y])
        angle = math.atan2(self.relative_y, self.relative_x)
        # gain = 1.
        self.get_logger().info(f"lookahead {lookahead}")

        turn_radius = lookahead / (2*math.sin(math.atan2(self.relative_y,self.relative_x)))

        if lookahead > 1.2 and abs(turn_radius) >= self.min_turn_radius and (curr_time-self.detection_valid_time <= self.detection_valid_thres):
            steer_angle = math.atan(self.wheelbase_length/turn_radius)

            self.drive_cmd(steer_angle, self.cmd_speed)
            self.get_logger().info('FORWARD, STEERING {steer_angle}')

        # elif abs(turn_radius) < self.min_turn_radius or abs(angle) > self.angle_thres or (curr_time-self.detection_valid_time > self.detection_valid_thres):
        elif abs(turn_radius) < self.min_turn_radius or abs(angle) > self.angle_thres and (curr_time-self.detection_valid_time <= self.detection_valid_thres):
            # go back and turn
            if self.relative_y > 0:
                # cone is to the left, go back right
                self.drive_cmd(-self.max_steer, -0.7)
                self.get_logger().info('FULL BACK RIGHT')
            else:
                # cone right, go back left
                self.drive_cmd(self.max_steer, -0.7)
                self.get_logger().info('FULL BACK LEFT')
        else:
            self.drive_cmd(0.0, 0.0)
            # the car stops moving for 5 seconds and then continues to move
            node_dir = os.path.dirname(os.path.abspath(__file__))
            img_path = os.path.join(node_dir, "outputs/banana.png")
            cv2.imwrite(img_path, self.image)
            if not self.start_time:
                 self.start_time = self.get_clock().now().nanoseconds*10**-9
            self.get_logger().info(f'start time is {self.start_time}')
            self.get_logger().info(f'current time is {self.get_clock().now().nanoseconds*10**-9}')
            if self.get_clock().now().nanoseconds*10**-9 - self.start_time < 5:
                return

            self.get_logger().info(f'end time is {self.get_clock().now().nanoseconds*10**-9}')
            ##if target distance is less than 0.5 m then send the switch command again

            self.get_logger().info(f'Publishing SHRINK ray count of {self.shrinkray_count} ')
            self.detection_valid_time = None
            self.shrink_count_cmd(self.shrinkray_count)
            #stop publishing
            # self.get_logger().info('turning self.detect to false')
            self.detect = False
            self.start_searching = False
            self.shrinkray_count = 2

        # self.get_logger().info('i am now capable of publishing commands')
        # if target_distance > 1.0:
        #     self.drive_cmd(steer_angle, self.cmd_speed)
        # else:
            # self.drive_cmd(steer_angle, 0.0)
            # # the car stops moving for 5 seconds and then continues to move
            # if not self.start_time:
            #      self.start_time = self.get_clock().now().nanoseconds*10**-9
            # self.get_logger().info(f'start time is {self.start_time}')
            # self.get_logger().info(f'current time is {self.get_clock().now().nanoseconds*10**-9}')
            # if self.get_clock().now().nanoseconds*10**-9 - self.start_time < 5:

            #     return

            # self.get_logger().info(f'end time is {self.get_clock().now().nanoseconds*10**-9}')
            # ##if target distance is less than 0.5 m then send the switch command again

            # self.get_logger().info(f'Publishing SHRINK ray count of {self.shrinkray_count} ')
            # self.shrink_count_cmd(self.shrinkray_count)
            # #stop publishing
            # self.get_logger().info('turning self.detect to false')
            # self.detect = False
            # self.shrinkray_count = 2

    def start_detection_callback(self, bool_msg):
        self.get_logger().info('bool msg received')
        if self.detect == False:
            # self.get_logger().info(f'turning self.detect to {bool_msg.data}')
            self.detect = bool_msg.data
            self.start_time = None

    def start_searching_callback(self, bool_msg):
        self.start_searching = bool_msg.data

    def drive_cmd(self, steer, speed = 1.0):

            drive_cmd_drive = AckermannDriveStamped()

            drive_cmd_drive.drive.speed = speed

            drive_cmd_drive.drive.steering_angle = steer

            drive_cmd_drive.drive.steering_angle_velocity = 0.0

            drive_cmd_drive.drive.acceleration = 0.0

            drive_cmd_drive.drive.jerk = 0.0

            drive_cmd_drive.header.stamp = self.get_clock().now().to_msg()

            self.drive_pub.publish(drive_cmd_drive)

    def switch_cmd(self, flag:bool):
        msg = Bool()
        msg.data = flag
        self.switch_pub.publish(msg)

    def shrink_count_cmd (self, count):
        msg = Int32()
        msg.data = count
        self.shrinkray_count_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
