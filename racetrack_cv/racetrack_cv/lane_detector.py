#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import cv2

from cv_bridge import CvBridge, CvBridgeError
from vs_msgs.msg import ConeLocationPixel, ConeLocation
from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs.msg import Image
# import imageio
from racetrack_cv.visualization_tools import VisualizationTools

from racetrack_cv.zed_source import zed_source

# from visual servoing lab
######################################################
PTS_IMAGE_PLANE = [[289, 263],
                   [71, 299],
                   [492, 229],
                   [562, 237],
                   [342, 217],
                   [606, 323],
                   [266, 294]]
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
PTS_GROUND_PLANE = [[21.65, 4.53],
                    [16.54, 15.16],
                    [30.12, -13.39],
                    [26.77, -17.32],
                    [35.43, 0],
                    [13.78, -9.84],
                    [16.93, 5.12]]
######################################################

METERS_PER_INCH = 0.0254

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.declare_parameter("white_lower_lims", [0, 0, 0])
        self.declare_parameter("white_upper_lims", [0, 0, 0])
        self.declare_parameter("focus_area", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("line_angle_limit", 0.0)
        self.declare_parameter("lane_width", 0.0)
        self.declare_parameter("lookahead", 1.0)
        self.declare_parameter("close_to_line_thres", 0.0)
        self.declare_parameter("image_fps", 60)
        self.declare_parameter("going_right", True)
        self.declare_parameter("offset", 0.0)

        self.white_lower_lims = self.get_parameter("white_lower_lims").value
        self.white_upper_lims = self.get_parameter("white_upper_lims").value
        self.focus_area = self.get_parameter("focus_area").value
        self.line_angle_limit = self.get_parameter("line_angle_limit").get_parameter_value().double_value
        self.lane_width = self.get_parameter("lane_width").get_parameter_value().double_value
        self.lookahead = self.get_parameter("lookahead").get_parameter_value().double_value
        self.close_to_line_thres = self.get_parameter("close_to_line_thres").get_parameter_value().double_value
        self.image_fps = self.get_parameter("image_fps").get_parameter_value().integer_value
        self.going_right = self.get_parameter("going_right").get_parameter_value().bool_value
        self.offset = self.get_parameter("offset").get_parameter_value().double_value

        self.get_logger().info(f'WHITE PIXEL HSV LIMITS: {self.white_lower_lims, self.white_upper_lims}')
        
        self.cone_pub = self.create_publisher(ConeLocation, "/relative_cone", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/markers", 1)
        self.debug_pub = self.create_publisher(Image, "/cv_debug_img", 10)
        # self.image_sub = self.create_subscription(Image, "camera_topic", self.image_callback, 5)
        self.image_timer = self.create_timer(1.0/self.image_fps, self.image_callback)
        self.bridge = CvBridge()

        self.zed = zed_source(set_fps=self.image_fps, resize_dim=(640, 360)) # TODO: check if homography works using that resolution or if scaling is different (disproportionate width/height? maybe ROS zed node was cropping and we need to do that too, check FoV)
        # self.video= imageio.get_writer('output_video.mp4', fps=self.image_fps)
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        # self.original_width = 720
        # self.original_height = 1280

        self.prev_to_right = False
        self.prev_to_left = False

        self.x_real = None
        self.y_real = None

        self.status = 0
        self.vid_duration=5
        self.step = 0

    # from visual servoing lab
    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]

        # self.get_logger().info(f"Pixel ({u},{v}) -> World ({x:.2f}, {y:.2f})")
        
        return x, y

    # def image_callback(self, img_msg):
    def image_callback(self):
        self.step+=1
        # get image and convert to HSV
        # image_cv = self.bridge.imgmsg_to_cv2(img_msg, "bgra8")

        # use ZED wrapper
        frame = self.zed.get_frame()
        image_cv = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # to keep rest of the code the same

        # just for testing with lower resolution
        # cv2.imshow('img', image_cv)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # image_cv = cv2.resize(image_cv, (672, 376))
        # cv2.imshow('img', image_cv)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        hsv_img = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
        self.width = hsv_img.shape[1]
        self.height = hsv_img.shape[0]
        # self.get_logger().info(f'{self.width, self.height}')

        # cut the image to the selected area
        min_x, min_y, max_x, max_y = self.focus_area
        min_x, max_x = int(min_x*hsv_img.shape[0]), int(max_x*hsv_img.shape[0])
        min_y, max_y = int(min_y*hsv_img.shape[1]), int(max_y*hsv_img.shape[1])
        focused_img = hsv_img[min_x:max_x, min_y:max_y]
        mask_bgr = image_cv[min_x:max_x, min_y:max_y].copy()
        # self.get_logger().info(f"{mask_bgr.shape}")

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
        # mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        # left_line, right_line = None, None # (r, theta, slope, offset)
        left_line_ground, right_line_ground = None, None # (theta_ground, slope_ground, offset_ground)
        # theta also gives us which lanes are to the left and which are to the right, to know which lane we are in
        if lines is not None:
            for i in range(len(lines)):
                r = lines[i][0][0]
                theta = lines[i][0][1]
                if theta != 0:
                    slope = -math.cos(theta)/math.sin(theta)
                    offset = r/math.sin(theta)
                else:
                    slope = 10000
                    offset = 10000
                if slope == 0:
                    continue
                
                if theta < 0:
                    theta += np.pi
                if theta > np.pi:
                    theta -= np.pi
                if theta > np.pi/2:
                    theta -= np.pi
                # self.get_logger().info(f'LINE {i} WITH POLAR PARAMS: {r, theta}')
                # discard if abs(theta) too large -> perpendicular to the track lines

                # project line onto ground plane
                # mask_bgr = cv2.line(mask_bgr, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (0.0, 0.0, 255.0), 5)
                line_x1_px, line_y1_px = int((mask.shape[0]-offset)/slope), mask.shape[0] # intersect with bottom of focused area (closest point)
                line_x2_px, line_y2_px = int(-offset/slope), 0 # intersect with top of focused area (furthest point) -> SHOULD BE ON THE GROUND
                
                line_x1_px += min_y
                line_x2_px += min_y
                line_y1_px += min_x
                line_y2_px += min_x
                closest = self.transformUvToXy(line_x1_px, line_y1_px)
                furthest = self.transformUvToXy(line_x2_px, line_y2_px)
                # self.get_logger().info(f'closest: {closest}, furthest: {furthest}')
                slope_ground = (furthest[1]-closest[1])/(furthest[0]-closest[0])
                offset_ground = closest[1]-slope_ground*closest[0]
                theta_ground = math.atan(slope_ground) # -pi/2 to pi/2 wrt to vertical/forwards
                # if theta_ground < 0:
                #     theta_ground += math.pi # zero to pi, wrt car horizontal
                # theta_ground -= math.pi/2.0 # -pi/2 to pi/2, wrt to vertical/forwards
                
                # self.get_logger().info(f'THETA GROUND: {theta_ground}, OFFSET GROUND: {offset_ground}')
                
                if abs(theta_ground) > self.line_angle_limit/180*np.pi:
                    continue
                
                
                if offset_ground > 0 and (left_line_ground is None or ((self.going_right and left_line_ground[0] > theta_ground) or ((not self.going_right) and left_line_ground[0] < theta_ground))):
                    left_line_ground = (theta_ground, slope_ground, offset_ground)
                    offset_left = offset_ground
                    theta_left = theta_ground
                    slope_img_left = slope
                    offset_img_left = offset
                    slope_left = slope_ground
                    closest_left = closest
                    furthest_left = furthest
                    # self.get_logger().info(f'FOUND ONE LEFT: {left_line_ground}')
                if offset_ground < 0 and (right_line_ground is None or ((self.going_right and right_line_ground[0] > theta_ground) or ((not self.going_right) and right_line_ground[0] < theta_ground))):
                    right_line_ground = (theta_ground, slope_ground, offset_ground)
                    offset_right = offset_ground
                    theta_right = theta_ground
                    slope_img_right = slope
                    offset_img_right = offset
                    slope_right = slope_ground
                    closest_right = closest
                    furthest_right = furthest
                    # self.get_logger().info(f'FOUND ONE RIGHT: {right_line_ground}')
                

                # if abs(theta) > self.line_angle_limit/180*np.pi:
                #     continue
                # # self.get_logger().info(f'LINE {i} VALID')
                # # mask_rgb = cv2.line(mask_rgb, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (0.0, 0.0, 255.0), 5)
                # if theta > 0 and (left_line is None or abs(left_line[1]) > abs(theta)):
                #     left_line = (r, theta, slope, offset)
                # if theta < 0 and (right_line is None or abs(right_line[1]) > abs(theta)):
                #     right_line = (r, theta, slope, offset)

            marker_arr = MarkerArray()

            if left_line_ground is not None:
                mask_bgr = cv2.line(mask_bgr, (0, int(offset_img_left)), (int(mask.shape[1]), int(slope_img_left*mask.shape[1]+offset_img_left)), (0.0, 0.0, 255.0), 5)
                marker_arr.markers.append(VisualizationTools.plot_line([closest_left[0], furthest_left[0]], [closest_left[1], furthest_left[1]], 0, (1.0, 0.0, 0.0)))
                # self.get_logger().info(f'LEFT LINE GROUND: {left_line_ground}, closest left: {closest_left}, furthest left: {furthest_left}')
            if right_line_ground is not None:
                mask_bgr = cv2.line(mask_bgr, (0, int(offset_img_right)), (int(mask.shape[1]), int(slope_img_right*mask.shape[1]+offset_img_right)), (0.0, 255.0, 0.0), 5)
                marker_arr.markers.append(VisualizationTools.plot_line([closest_right[0], furthest_right[0]], [closest_right[1], furthest_right[1]], 1, (0.0, 1.0, 0.0)))
                # self.get_logger().info(f'RIGHT LINE GROUND: {right_line_ground}, closest right: {closest_right}, furthest right: {furthest_right}')

            # if left_line is not None:
            #     r, theta, slope, offset = left_line
            #     mask_bgr = cv2.line(mask_bgr, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (0.0, 0.0, 255.0), 5)
            #     left_line_x1_px, left_line_y1_px = int((mask.shape[0]-offset)/slope), mask.shape[0] # intersect with bottom of focused area (closest point)
            #     left_line_x2_px, left_line_y2_px = int(-offset/slope), 0 # intersect with top of focused area (furthest point) -> SHOULD BE ON THE GROUND
            #     # mask_bgr = cv2.circle(mask_bgr, (left_line_x1_px, left_line_y1_px), 5, (0.0, 255.0, 0.0), 5)
            #     # mask_bgr = cv2.circle(mask_bgr, (left_line_x2_px, left_line_y2_px), 5, (255.0, 0.0, 0.0), 5)
            #     left_line_x1_px += min_y
            #     left_line_x2_px += min_y
            #     left_line_y1_px += min_x
            #     left_line_y2_px += min_x
            #     closest_left = self.transformUvToXy(left_line_x1_px, left_line_y1_px)
            #     furthest_left = self.transformUvToXy(left_line_x2_px, left_line_y2_px)
            #     slope_left = (furthest_left[1]-closest_left[1])/(furthest_left[0]-closest_left[0])
            #     offset_left = closest_left[1]-slope_left*closest_left[0]
            #     # self.get_logger().info(f'LEFT LINE PROJECTED PARAMS: {slope_left, offset_left}')
            #     marker_arr.markers.append(VisualizationTools.plot_line([closest_left[0], furthest_left[0]], [closest_left[1], furthest_left[1]], 0, (1.0, 0.0, 0.0)))
            # if right_line is not None:
            #     r, theta, slope, offset = right_line
            #     mask_bgr = cv2.line(mask_bgr, (0, int(offset)), (int(mask.shape[1]), int(slope*mask.shape[1]+offset)), (0.0, 255.0, 0.0), 5)
            #     right_line_x1_px, right_line_y1_px = int((mask.shape[0]-offset)/slope), mask.shape[0] # intersect with bottom of focused area (closest point)
            #     right_line_x2_px, right_line_y2_px = int(-offset/slope), 0 # intersect with top of focused area (furthest point) -> SHOULD BE ON THE GROUND
            #     # mask_bgr = cv2.circle(mask_bgr, (right_line_x1_px, right_line_y1_px), 5, (0.0, 255.0, 0.0), 5)
            #     # mask_bgr = cv2.circle(mask_bgr, (right_line_x2_px, right_line_y2_px), 5, (255.0, 0.0, 0.0), 5)
            #     right_line_x1_px += min_y
            #     right_line_x2_px += min_y
            #     right_line_y1_px += min_x
            #     right_line_y2_px += min_x
            #     closest_right = self.transformUvToXy(right_line_x1_px, right_line_y1_px)
            #     furthest_right = self.transformUvToXy(right_line_x2_px, right_line_y2_px)
            #     slope_right = (furthest_right[1]-closest_right[1])/(furthest_right[0]-closest_right[0])
            #     offset_right = closest_right[1]-slope_right*closest_right[0]
            #     # self.get_logger().info(f'RIGHT LINE PROJECTED PARAMS: {slope_right, offset_right}')
            #     marker_arr.markers.append(VisualizationTools.plot_line([closest_right[0], furthest_right[0]], [closest_right[1], furthest_right[1]], 1, (0.0, 1.0, 0.0)))

            if left_line_ground is not None or right_line_ground is not None:
                close_left = False
                close_right = False
                far_left = False
                far_right = False
                to_left = False
                to_right = False
                dist_left = None
                dist_right = None

                if left_line_ground is not None:
                    dist_left = abs(offset_left)/math.sqrt(offset_left**2+slope_left**2+1)
                    if dist_left < self.close_to_line_thres*self.lane_width:
                        close_left = True
                    if dist_left > (1-self.close_to_line_thres)*self.lane_width:
                        far_left = True
                if right_line_ground is not None:
                    dist_right = abs(offset_right)/math.sqrt(offset_right**2+slope_right**2+1)
                    if dist_right < self.close_to_line_thres*self.lane_width:
                        close_right = True
                    if dist_right > (1-self.close_to_line_thres)*self.lane_width:
                        far_right = True

                if close_right or far_left:
                    to_right = True
                
                if close_left or far_right:
                    to_left = True

                # if self.prev_to_left and to_right:
                #     # went left
                #     self.status -= 1
                # elif self.prev_to_right and to_left:
                #     # went right
                #     self.status += 1

                # if dist_left is not None and dist_left > (1-self.close_to_line_thres)*self.lane_width and self.prev_dist_left is not None and self.prev_dist_left < self.close_to_line_thres*self.lane_width:
                #     # went left
                #     self.status -= 1
                # elif dist_right is not None and dist_right > (1-self.close_to_line_thres)*self.lane_width and self.prev_dist_right is not None and self.prev_dist_right < self.close_to_line_thres*self.lane_width:
                #     # went right
                #     self.status += 1

                # logic for finding goal point between lines (or offset from one if only one found) using the projected lanes
                # self.get_logger().info(f'LANE: {self.status}')
                # self.get_logger().info(f'TO LEFT: {to_left}')
                # self.get_logger().info(f'TO RIGHT: {to_right}')
                # self.get_logger().info(f'PREVIOUSLY TO LEFT: {self.prev_to_left}')
                # self.get_logger().info(f'PREVIOUSLY TO RIGHT: {self.prev_to_right}')
                if self.status == 0:
                    # stayed in lane
                    if left_line_ground is not None and right_line_ground is not None:
                        # find middle line
                        # slope_mid = (slope_left+slope_right)/2
                        theta_mid = (theta_left+theta_right)/2
                        slope_mid = math.tan(theta_mid)
                        offset_mid = (offset_left+offset_right)/2
                        # offset_mid = offset_mid - self.offset*math.sqrt(slope_mid**2+1)
                        offset_mid = offset_mid - self.offset
                    elif right_line_ground is None:
                        # only left, offset by distance
                        slope_mid = slope_left
                        # offset_mid = offset_left - (self.lane_width/2.0+self.offset)*math.sqrt(slope_mid**2+1)
                        offset_mid = offset_left - (self.lane_width/2.0)*math.sqrt(slope_mid**2+1) - self.offset
                    elif left_line_ground is None:
                        # only right
                        slope_mid = slope_right
                        # offset_mid = offset_right + (self.lane_width/2.0-self.offset)*math.sqrt(slope_mid**2+1)/2
                        offset_mid = offset_right + (self.lane_width/2.0)*math.sqrt(slope_mid**2+1) - self.offset
                elif self.status < 0:
                    # went left, put goal to right
                    if left_line_ground is not None and right_line_ground is not None:
                        # find middle line
                        slope_mid = (slope_left+slope_right)/2
                        offset_mid = (offset_left+offset_right)/2 - self.lane_width*math.sqrt(slope_mid**2+1)
                    elif right_line_ground is None:
                        # only left, offset by distance
                        slope_mid = slope_left
                        offset_mid = offset_left - 3*self.lane_width*math.sqrt(slope_mid**2+1)/2
                    elif left_line_ground is None:
                        # only right
                        slope_mid = slope_right
                        offset_mid = offset_right - self.lane_width*math.sqrt(slope_mid**2+1)/2
                else:
                    # went right, put goal to left
                    if left_line_ground is not None and right_line_ground is not None:
                        # find middle line
                        slope_mid = (slope_left+slope_right)/2
                        offset_mid = (offset_left+offset_right)/2 + self.lane_width*math.sqrt(slope_mid**2+1)
                    elif right_line_ground is None:
                        # only left, offset by distance
                        slope_mid = slope_left
                        offset_mid = offset_left + self.lane_width*math.sqrt(slope_mid**2+1)/2
                    elif left_line_ground is None:
                        # only right
                        slope_mid = slope_right
                        offset_mid = offset_right + 3*self.lane_width*math.sqrt(slope_mid**2+1)/2

                # lookahead is from start of the line not from robot, to hopefully make recovery smoother (also easier math for me)
                self.x_real, self.y_real = self.lookahead/math.sqrt(slope_mid**2+1), slope_mid*self.lookahead/math.sqrt(slope_mid**2+1)+offset_mid

                marker_arr.markers.append(VisualizationTools.plot_line([0.0, self.x_real], [offset_mid, self.y_real], 2, (0.0, 0.0, 1.0)))

                marker_arr.markers.append(self.draw_marker(self.x_real, self.y_real, "base_link", 3))

                self.prev_to_left = to_left
                self.prev_to_right = to_right

                if dist_left is not None:
                    self.prev_dist_left = dist_left
                if dist_right is not None:
                    self.prev_dist_right = dist_right
            
            if self.x_real is not None:
                relative_xy_msg = ConeLocation()
                relative_xy_msg.x_pos = self.x_real
                relative_xy_msg.y_pos = self.y_real
                self.cone_pub.publish(relative_xy_msg)

            self.marker_pub.publish(marker_arr)


        # debug_msg = self.bridge.cv2_to_imgmsg(mask, "8UC1")
        # debug_msg = self.bridge.cv2_to_imgmsg(mask_rgb, "rgb8")
        debug_msg = self.bridge.cv2_to_imgmsg(mask_bgr, "bgr8")

        self.debug_pub.publish(debug_msg)
        # self.get_logger().info(f"{mask_bgr.shape}")
        # mask_rgb=cv2.cvtColor(mask_bgr, cv2.COLOR_BGR2RGB)
        # mask_rgb = mask_rgb.astype('uint8')

        # if self.step<self.vid_duration*self.image_fps:
        #     self.video.append_data(frame)

        # self.get_logger().info(f"{self.step}")
        # if self.step==self.vid_duration*self.image_fps:
        #     self.video.release()
        #     self.get_logger().info('video saved')
    # from visual servoing lab
    def draw_marker(self, cone_x, cone_y, message_frame, id):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.id = id
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        return marker
def destroy_node(self):
    if self.video is not None:
        self.video.close()
        self.get_logger().info('video saved')
    super().destroy_node()
def main():
    rclpy.init()
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()