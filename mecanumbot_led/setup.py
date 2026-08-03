from setuptools import find_packages, setup

package_name = "mecanumbot_led"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fortuz",
    maintainer_email="fortuz19@gmail.com",
    description="Package for the Mecanumbot led",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mecanumbot_led_service = mecanumbot_led.led_control_node:main",
        ],
    },
)
