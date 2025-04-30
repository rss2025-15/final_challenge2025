import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from vs_msgs.msg import ConeLocation, ConeLocationPixel
import numpy as np
import math
# from .detector import Detector

#In real world i am =assuming opinits give no n rviz would be here right?
# so we should have an idea of what x and y we are heading towards
# this would help me find whatever is needed for model type and see if I detect then I can kick in rigjt
#this node does not even need any homography transformation what soeevr, sohoudl eb able to get point from controller/
#trajectory follower lets see how the integration happens
class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        # self.detector = Detector()
        # self.publisher = None #TODO
        # self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        # self.bridge = CvBridge()
        ####
        self.drive_topic = "/drive"
        self.lookahead = 2.0  # FILL IN #
        self.speed = 0.5  # FILL IN #
        self.wheelbase_length = .46  # FILL IN #
        self.odom_topic =  "/odom"
        self.target_x = None
        self.target_y = None
        self.start_time = None

        self.bridge = CvBridge()

        ####


        ####


        ###subscribers

        self.subscriber = self.create_subscription(PointStamped, "/clicked_point", self.callback, 1)
        self.robot_pose_subscriber = self.create_subscription(ConeLocation, '/estimated_robot', self.pose_callback, 1)
        self.homography_subscriber = self.create_subscription(PoseStamped, '/relative_cone', self.homography_callback, 1)
        self.camera_subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.camera_callback, 1)

        ###publishers
        self.pixel_publisher = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        ###

        ####
        self.get_logger().info("Detector Initialized")
# odom_topic:

    # def callback(self, img_msg):
    #     # Process image with CV Bridge
    #     # image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

    #     #TODO:
    #     1

    #when should we put the relative cone_px angle? it should be done during the camera callback actually i THINK


    def callback(self, point_msg: PointStamped):
        # Process image with CV Bridge
        # image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #TODO:
        #everything will initially be in world frame
        #i will need to change from world frame to real frame
        x,y = point_msg.point.x, point_msg.point.y

        #lets convert these to robot frame now
        #lets get the robot position here as well from base link
        self.target_x = x
        self.target_y = y

        self.get_logger().info(f'target x received as {self.target_x}')
        self.get_logger().info(f'target y received as {self.target_y}')

    def camera_callback(self, camera_msg):

        # extract the image out of camera msg
        image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")

        #one we have the image we can run our detector to give bounding box of our image
        cone_pixel = ConeLocationPixel()
        cone_pixel.u = float(x_pixel)
        cone_pixel.v = float(y_pixel)
        self.pixel_publisher.publish(cone_pixel)


        #assume we finally have the x and y pixels

    def homography_callback(self, position_msg: ConeLocation ):
        self.target_x = position_msg.x_pos
        self.target_y = position_msg.y_pos
        self.get_logger().info(f'target x received as {self.target_x}')
        self.get_logger().info(f'target y received as {self.target_y}')

        #pose call back should continue working as it is right now


    def pose_callback(self, estimated_robot_msg):
        # self.get_logger().info(f'in pose call back')
        #return if target x and target y not received
        if not (self.target_x or self.target_y):
            return


        orientation = euler_from_quaternion([
        estimated_robot_msg.pose.orientation.x,
        estimated_robot_msg.pose.orientation.y,
        estimated_robot_msg.pose.orientation.z,
        estimated_robot_msg.pose.orientation.w
        ])[-1]

        robot_pose = np.array([
            estimated_robot_msg.pose.position.x,
            estimated_robot_msg.pose.position.y,
            orientation
        ])

        pts = np.array([self.target_x, self.target_y])
        lookahead_angle = math.atan2(pts[1]-robot_pose[1], pts[0] - robot_pose[0])
        angle_in_robot_frame = self.relative_angle_rad(lookahead_angle, robot_pose[2])

        #lets find distance between robot and target point
        target_distance = math.sqrt((robot_pose[0]-self.target_x)**2 + (robot_pose[1] - self.target_y)**2)
        self.get_logger().info(f'distance of robot from target point is {target_distance}')

        turn_radius = target_distance / (2*math.sin(angle_in_robot_frame))
        steer_angle = math.atan(self.wheelbase_length/turn_radius)
        self.cmd_speed = 1.0


        if target_distance > 0.5:
            self.drive_cmd(steer_angle, self.cmd_speed)
        else:
            #the car stops moving for 5 seconds and then continues to move
            if not self.start_time:
                 self.start_time = self.get_clock().now().nanoseconds*10**-9
            self.get_logger().info(f'start time is {self.start_time}')
            self.get_logger().info(f'current time is {self.get_clock().now().nanoseconds*10**-9}')
            if self.get_clock().now().nanoseconds*10**-9 - self.start_time < 5:
                self.drive_cmd(steer_angle, 0.0)
            else:
                 self.drive_cmd(steer_angle, self.cmd_speed)

    def normalize_angle_rad(self, angle):
            """ Normalize angle to [-π, π) radians """
            return (angle + np.pi) % (2 * np.pi) - np.pi

    def relative_angle_rad(self, object_angle, robot_angle):
            return self.normalize_angle_rad(object_angle - robot_angle)

    def drive_cmd(self, steer, speed = 1.0):


            drive_cmd_drive = AckermannDriveStamped()

            drive_cmd_drive.drive.speed = speed

            drive_cmd_drive.drive.steering_angle = steer

            drive_cmd_drive.drive.steering_angle_velocity = 0.0

            drive_cmd_drive.drive.acceleration = 0.0

            drive_cmd_drive.drive.jerk = 0.0

            drive_cmd_drive.header.stamp = self.get_clock().now().to_msg()

            self.drive_pub.publish(drive_cmd_drive)


def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
