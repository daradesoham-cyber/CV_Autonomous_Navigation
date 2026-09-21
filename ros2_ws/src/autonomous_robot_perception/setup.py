import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'autonomous_robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soham Darade',
    maintainer_email='soham@example.com',
    description='Computer Vision and LiDAR-Camera Fusion Nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = autonomous_robot_perception.camera_node:main',
            'object_detection_node = autonomous_robot_perception.object_detection_node:main',
            'lidar_camera_fusion_node = autonomous_robot_perception.lidar_camera_fusion_node:main',
            'navigation_perception_node = autonomous_robot_perception.navigation_perception_node:main',
            'navigation_controller_node = autonomous_robot_perception.navigation_controller_node:main',
            'sign_detection_node = autonomous_robot_perception.sign_detection_node:main',
        ],
    },
)
