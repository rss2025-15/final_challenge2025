from setuptools import find_packages, setup

package_name = 'final_challenge_stata'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_builder = final_challenge_stata.trajectory_builder:main',
            'trajectory_loader = final_challenge_stata.trajectory_loader:main',
            'trajectory_follower = final_challenge_stata.trajectory_follower:main',
            'trajectory_planner_astar = final_challenge_stata.trajectory_planner_astar:main',
            'trajectory_planner_rrt = final_challenge_stata.trajectory_planner_rrt:main',
            'basement_point_publisher = final_challenge_stata.basement_point_publisher:main',
            'controller_control = final_challenge_stata.controller_control:main'
        ],
    },
)
