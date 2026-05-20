from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanumbot_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fortuz',
    maintainer_email='fortuz19@gmail.com',
    description='Audio input handler that streams microphone PCM data to ROS.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'input_handler = mecanumbot_audio.input_handler:main',
        ],
    },
)
