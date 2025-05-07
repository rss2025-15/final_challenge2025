**launch order:**
teleop 
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed 
ros2 launch final_challenge_stata path.launch.xml 
ros2 launch racecar_simulator localization_simulate.launch.xml 

scp -r racecar@192.168.1.18:/home/racecar/racecar_ws/src/final_challenge2025/final_challenge_stata/rosbags_stata/racetrack_panos_fucked_up/rosbag2_2025_05_04-19_43_24 /Users/jingcao/Desktop