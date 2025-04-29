import os
import glob
from setuptools import find_packages, setup

package_name = 'racetrack_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/visual_servoing/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='jcao0715@gmail.com',
    description='Final Challenge 2025 controller package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_follower = racetrack_controller.track_follower:main',
        ],
    },
)
