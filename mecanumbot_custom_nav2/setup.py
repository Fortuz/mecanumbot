from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_custom_nav2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Csenge Hubay",
    maintainer_email="csengehubay@gmail.com",
    description=(
        "Map analysis for Mecanumbot's seeking system: the RRT frontier "
        "detector, the occupancy model, the T1 exit criteria, and the handler "
        "that turns the Deep3R server's 2D/3D verdict into a nav2 keepout "
        "mask. No node here commands motion."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # The one node in this package. The explorer that used to live here
            # moved to mecanumbot_autoslam in mecanumbot_behaviours: it sends
            # the robot places, and this package no longer commands motion.
            "mecanumbot_map_agreement_node = "
            "mecanumbot_custom_nav2.map_agreement_node:main",
        ],
    },
)
