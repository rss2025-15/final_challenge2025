import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Pose, PoseArray
from std_msgs.msg import Int32, Bool

class Control(Node):
    # "what the hell is going on"


    def __init__(self):
        super().__init__("controller_control")

        self.shinkray_point_sub = self.create_subscription(
            PoseArray,
            "/shrinkray_part", # from basement_point_pub
            self.shrinkray_point_cb,
            1
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.pose_cb,
            10
        )

        self.shrinkray_count_sub = self.create_subscription(
            Int32,
            "/shrinkray_count", # implement; increment in detection once shrinkray detected
            self.shrinkray_count_cb,
            1
        )


        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            "/waypoint",
            1
        )

        self.waypoints = [PoseStamped(), PoseStamped(), PoseStamped()]

    def shrinkray_point_cb(self, shrinkray_point_msg):
        # TA clicks points

        for i, pose in enumerate(shrinkray_point_msg.poses):
            self.waypoints[i].header = shrinkray_point_msg.header
            self.waypoints[i].pose = pose
        
        self.waypoint_pub.publish(self.waypoints[0])
    
    def pose_cb(self, pose_msg):
        # initial pose estimate --> our start pose

        self.waypoints[2].header = pose_msg.header
        self.waypoints[2].pose = pose_msg.pose.pose
    
    def shrinkray_count_cb(self, shrinkray_count_msg):
        # after 5 second stop at shrinkray

        new_path_goal = shrinkray_count_msg.data
        self.waypoint_pub.publish(self.waypoints[new_path_goal])

    

