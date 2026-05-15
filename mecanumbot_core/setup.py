from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanumbot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fortuz',
    maintainer_email='fortuz19@gmail.com',
    description='Nodes and linraries generating direct connection with the openCR board for mecanum wheeled robots and sensors',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'mecanumbot_io_node = mecanumbot_core.mecanumbot_IO_node:main',
          'mecanumbot_sensorproc_node = mecanumbot_core.mecanumbot_sensorproc_node:main',
          'mecanumbot_sim_io_node = mecanumbot_core.mecanumbot_sim_io_node:main',
          'mecanumbot_sim_oracle_subject_node = mecanumbot_core.mecanumbot_sim_oracle_subject_node:main',
          'mecanumbot_sim_behavior_evaluator_node = mecanumbot_core.mecanumbot_sim_behavior_evaluator_node:main',
          'mecanumbot_sim_detection_evaluator_node = mecanumbot_core.mecanumbot_sim_detection_evaluator_node:main',
          'mecanumbot_sim_visualization_node = mecanumbot_core.mecanumbot_sim_visualization_node:main',
          'mecanumbot_sim_detector_debug_node = mecanumbot_core.mecanumbot_sim_detector_debug_node:main',
          'mecanumbot_sim_nav_shim_node = mecanumbot_core.mecanumbot_sim_nav_shim_node:main',
          'mecanumbot_battery_alert = mecanumbot_core.mecanumbot_battery_alert:main',              ],
    },
)
