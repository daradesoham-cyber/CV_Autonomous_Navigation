import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'hospital_logistics'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soham Darade',
    maintainer_email='soham@example.com',
    description='V3 hospital logistics layer',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
