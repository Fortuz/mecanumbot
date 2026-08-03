from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_monitor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="csenge",
    maintainer_email="csengehubay@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "wheel_monitor = mecanumbot_monitor.wheel_monitor:main",
            "accessory_monitor = mecanumbot_monitor.accessory_monitor:main",
            "rosbag_stepper = mecanumbot_monitor.rosbag_stepper:main",
            "playback_stepper = mecanumbot_monitor.rosbag_stepper:main",
            "metric_eval = mecanumbot_monitor.metric_eval:main",
            "metric_logger = mecanumbot_monitor.metric_eval:main",
        ],
    },
)
