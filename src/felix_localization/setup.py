import os
from glob import glob

from setuptools import find_packages, setup

package_name = "felix_localization"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kevin Williams",
    maintainer_email="kevin.williams@sidusa.com",
    description="robot_localization EKF fusing wheel odometry + IMU for the Felix robot.",
    license="Proprietary",
    entry_points={
        "console_scripts": [],
    },
)
