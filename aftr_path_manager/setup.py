# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Setup configuration for aftr_path_manager."""


# [User Code Begin]
from glob import glob
import os
# [User Code End]

from setuptools import find_packages
from setuptools import setup

package_name = "aftr_path_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        # [User Code Begin]
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        # [User Code End]
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pyo",
    maintainer_email="inpyoi1304@gmail.com",
    description=(
        "Autonomous Following and Towing Robot odometry path recording "
        "and pose saving node."
    ),
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            # [User Code Begin]
            "path_manager = "
            "aftr_path_manager.path_manager_node:main",
            # [User Code End]
        ],
    },
)
