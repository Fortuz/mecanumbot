import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "sim_scenarios"),
            glob("config/sim_scenarios/*.yaml"),
        ),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "mujoco"), glob("mujoco/*.xml")),
    ]
    + [
        # One entry per Gazebo model directory: model.sdf plus its manifest.
        (
            os.path.join("share", package_name, "models", os.path.basename(model_dir)),
            glob(os.path.join(model_dir, "*")),
        )
        for model_dir in sorted(glob("models/*"))
        if os.path.isdir(model_dir)
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="csenge",
    maintainer_email="csengehubay@gmail.com",
    description="Simulation backends (MuJoCo, Gazebo Sim) and evaluation nodes for the mecanumbot platform",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mecanumbot_sim_mujoco_io_node = mecanumbot_sim.mujoco_io_node:main",
            "mecanumbot_sim_gz_io_node = mecanumbot_sim.gz_io_node:main",
            "mecanumbot_sim_oracle_subject_node = mecanumbot_sim.oracle_subject_node:main",
            "mecanumbot_sim_behavior_evaluator_node = mecanumbot_sim.behavior_evaluator_node:main",
            "mecanumbot_sim_detection_evaluator_node = mecanumbot_sim.detection_evaluator_node:main",
            "mecanumbot_sim_visualization_node = mecanumbot_sim.visualization_node:main",
            "mecanumbot_sim_detector_debug_node = mecanumbot_sim.detector_debug_node:main",
            "mecanumbot_sim_nav_shim_node = mecanumbot_sim.nav_shim_node:main",
        ],
    },
)
