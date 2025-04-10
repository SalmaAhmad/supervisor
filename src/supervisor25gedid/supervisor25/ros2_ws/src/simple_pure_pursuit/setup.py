#!/usr/bin/env python3
"""Setup for the simple_pure_pursuit package."""
import os
from setuptools import find_packages, setup
from glob import glob
from setuptools import find_packages, setup

# pylint: disable=invalid-name
package_name = "simple_pure_pursuit"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share",package_name,"json"),glob("json/*.json"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mohamedalaa",
    maintainer_email="mohammed.alaa200080@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simple_pp_node = simple_pure_pursuit.simple_pp_node:main",
            "waypoints_node = simple_pure_pursuit.scripts.waypoints_node:main",
        ],
    },
)
