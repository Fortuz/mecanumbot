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
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Csenge Hubay",
    maintainer_email="csengehubay@gmail.com",
    description=(
        "RRT frontier exploration for Mecanumbot's autoslam pass, "
        "with the Deep3R reconstruction in the exit criteria."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mecanumbot_frontier_explorer_node = "
            "mecanumbot_custom_nav2.frontier_explorer_node:main",
            "mecanumbot_map_agreement_node = "
            "mecanumbot_custom_nav2.map_agreement_node:main",
        ],
    },
)
