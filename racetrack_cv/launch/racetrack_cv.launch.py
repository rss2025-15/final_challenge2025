import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    lane_detector_params = os.path.join(
        get_package_share_directory('racetrack_cv'),
        'config',
        'lane_detector_params.yaml'
        )
        
    lane_detector=Node(
        package = 'racetrack_cv',
        executable = 'lane_detector',
        parameters = [lane_detector_params],
        remappings = [
            ("camera_topic", "/zed/zed_node/rgb/image_rect_color")
        ]
    )

    homography_transformer=Node(
        package = 'visual_servoing',
        executable = 'homography_transformer',
        remappings = [
            
        ]
    )
    return LaunchDescription([
        lane_detector,
        # homography_transformer,
    ])