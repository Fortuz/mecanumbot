from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanumbot_camera_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fortuz',
    maintainer_email='fortuz19@gmail.com',
    description='Camera image publisher for USB and CSI cameras on NVIDIA Orin Nano.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_image_publisher_node = mecanumbot_camera_stream.camera_image_publisher_node:main',
            'compressed_camera_publisher_node = mecanumbot_camera_stream.compressed_camera_publisher_node:main',
            'h264_camera_publisher_node = mecanumbot_camera_stream.h264_camera_publisher_node:main'
        ],
    },
)
