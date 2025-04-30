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
        'final_challenge2025.final_challenge_stata.pickup_controller'

        #  'motion_model_test = localization.test.motion_model_test:main',
        #  'detection_node = localization.detection_node:main',
        ],
    },
)
