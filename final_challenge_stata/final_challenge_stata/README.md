**launch order:**
teleop
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
ros2 launch localization localize.launch.xml
ros2 launch final_challenge_stata path.launch.xml
ros2 launch racecar_simulator localization_simulate.launch.xml