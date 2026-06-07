import os
from glob import glob

from setuptools import find_packages, setup

package_name = "felix_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kevin Williams",
    maintainer_email="kevin.williams@sidusa.com",
    description="Base driver, teleop, and mecanum kinematics for the custom Felix robot.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "bridge = felix_base.bridge_node:main",
            "tof = felix_base.tof_array_node:main",
            "teleop = felix_base.teleop:main",
            "calibrate = felix_base.calibrate:main",
        ],
    },
)
