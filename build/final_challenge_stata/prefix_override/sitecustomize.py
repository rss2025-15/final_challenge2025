import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/racecar_ws/src/final_challenge2025/install/final_challenge_stata'
