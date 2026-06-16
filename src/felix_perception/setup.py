import os
from glob import glob

from setuptools import find_packages, setup

package_name = "felix_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
        # Ship the small .pt weights; the device-specific .engine is built by the
        # build-engine script into a writable cache and is gitignored (not shipped).
        (os.path.join("share", package_name, "models"),
            glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kevin Williams",
    maintainer_email="kevin.williams@sidusa.com",
    description="YOLO detection + lidar-fused map placement for the Felix robot.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "detector = felix_perception.detector_node:main",
            "fusion = felix_perception.fusion_node:main",
            "build-engine = felix_perception.build_engine:main",
            "calibrate-camera = felix_perception.calibrate_camera:main",
        ],
    },
)
