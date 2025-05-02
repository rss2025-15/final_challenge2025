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


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")

        ####
        self.drive_topic = "/vesc/low_level/input/navigation"
        self.lookahead = 2.0  # FILL IN #
        self.speed = 0.5  # FILL IN #
        self.wheelbase_length = .46  # FILL IN #
        self.odom_topic =  "/odom"
        self.target_x = None
        self.target_y = None
        self.start_time = None

        self.bridge = CvBridge()

        ###Detector
        self.detector = Detector()
        self.detector.set_threshold(0.1)
        ####
        self.send_drive_command = True
        self.last_switch_command = True
        self.detect = True
        # switch
        # Bool, /switch_parking
        # /shrinkray_count
        # / Int32



        ####


        ###subscribers

        self.homography_subscriber = self.create_subscription(ConeLocation, '/relative_cone', self.homography_callback, 1)
        self.camera_subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.camera_callback, 1)
        self.start_detection_subscriber = self.create_subscription(Bool, 'start_detection',self.start_detection_callback,  1)

        ###publishers
        self.pixel_publisher = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.switch_pub = self.create_publisher(Bool, '/switch_parking', 1)
        self.shrinkray_count_pub = self.create_publisher(Int32, '/shrinkray_count', 1)
        ###

        ####
        self.get_logger().info("Detector Initialized")



    def camera_callback(self, camera_msg):
        # return if we are not supposed to be detecting right now
        if not self.detect:
            return

        # extract the image out of camera msg
        image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        #pass this image to the detector for predictions
        results = self.detector.predict(image)
        predictions = results["predictions"]
        # self.get_logger().info('outside loop')
        #format
        #((356.390625, 110.8359375, 448.921875, 171.703125), 'banana')
        for prediction in predictions:
            self.get_logger().info(f'predicion is {prediction}')
            if prediction[1] == 'banana':
                #if last switch command was True only then publish message to switch

                #turn off the regular follower
                self.switch_cmd(False)




                # detector returns (xmin, ymin, xmax, ymax)
                x1_f, y1_f, x2_f, y2_f = prediction[0]



                x1, y1, x2, y2 = map(int, (x1_f, y1_f, x2_f, y2_f))
                self.get_logger().info(f'rectangle_box is {x1,x2,y1,y2}'  )




                middle_x_pixel = (x1 + x2) // 2
                bottom_y_pixel = y2
                #####debug
                # add box around orig image
                cv2.rectangle(image, (x1,y1), (x2, y2), (0,255,0), 2)
                cv2.putText(image, "FPS: not determined ", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                # self.get_logger().info(f"{cone_pixel.u, cone_pixel.v}")
                self.debug_pub.publish(debug_msg)

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
        self.target_x = position_msg.x_pos
        self.target_y = position_msg.y_pos
        self.get_logger().info(f'target x received as {self.target_x}')
        self.get_logger().info(f'target y received as {self.target_y}')

        #pose call back should continue working as it is right now

        #lets find distance between robot and target point
        target_distance = math.sqrt((self.target_x)**2 + (self.target_y)**2)
        self.get_logger().info(f'distance of robot from target point is {target_distance}')





        angle_in_robot_frame = math.atan2(self.target_y, self.target_x)
        self.get_logger().info(f'angle in robot frame is  is {angle_in_robot_frame}')

        turn_radius = target_distance / (2*math.sin(angle_in_robot_frame))
        steer_angle = math.atan(self.wheelbase_length/turn_radius)
        self.cmd_speed = 1.0

        self.get_logger().info('i am now capable of publishing commands')
        if target_distance > 0.5:
            self.drive_cmd(steer_angle, self.cmd_speed)
        else:
            ##if target distance is less than 0.5 m then send the switch command again
            self.shrink_count_cmd(1)
            #stop publishing
            self.detect = False






            #the car stops moving for 5 seconds and then continues to move
            # if not self.start_time:
            #      self.start_time = self.get_clock().now().nanoseconds*10**-9
            # self.get_logger().info(f'start time is {self.start_time}')
            # self.get_logger().info(f'current time is {self.get_clock().now().nanoseconds*10**-9}')
            # if self.get_clock().now().nanoseconds*10**-9 - self.start_time < 5:
            #     self.drive_cmd(steer_angle, 0.0)
            # else:
            #      self.drive_cmd(steer_angle, self.cmd_speed)


    def start_detection_callback(self, bool_msg):
        self.detect = bool_msg.data


    def drive_cmd(self, steer, speed = 1.0):


            drive_cmd_drive = AckermannDriveStamped()

            drive_cmd_drive.drive.speed = speed

            drive_cmd_drive.drive.steering_angle = steer

            drive_cmd_drive.drive.steering_angle_velocity = 0.0

            drive_cmd_drive.drive.acceleration = 0.0

            drive_cmd_drive.drive.jerk = 0.0

            drive_cmd_drive.header.stamp = self.get_clock().now().to_msg()

            self.drive_pub.publish(drive_cmd_drive)


    def switch_cmd(self, bool):
        msg = Bool()
        msg.data = bool
        self.switch_pub.publish(msg)


    def shrink_count_cmd (self, count):
        msg = Int32()
        msg.data = count
        self.shrinkray_count_pub.publisg(msg)


def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
