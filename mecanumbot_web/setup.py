import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_web"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    # Jinja templates live inside the Python package so Flask(__name__) finds
    # them; they need package_data to survive installation.
    package_data={package_name: ["templates/*.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="csenge",
    maintainer_email="csengehubay@gmail.com",
    description="Robot-hosted web GUI: joystick profiles, topic-rate diagnostics, "
                "behaviour parameters",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mecanumbot_web_node = mecanumbot_web.web_node:main",
        ],
    },
)
