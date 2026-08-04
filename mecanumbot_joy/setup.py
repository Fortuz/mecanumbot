import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_joy"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="csenge",
    maintainer_email="csengehubay@gmail.com",
    description="Onboard joystick control driven by a YAML mapping profile",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mecanumbot_joy_node = mecanumbot_joy.joy_node:main",
        ],
    },
)
