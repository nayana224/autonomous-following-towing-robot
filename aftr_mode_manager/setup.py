# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Setup configuration for aftr_mode_manager."""


# [User Code Begin]
import os
from glob import glob
# [User Code End]

from setuptools import find_packages
from setuptools import setup

package_name = "aftr_mode_manager"

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
        ("share/" + package_name, ["README.md"]),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        # [User Code End]
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pyo",
    maintainer_email="inpyoi1304@gmail.com",
    description="Autonomous Following and Towing Robot mode state and transition manager.",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            # [User Code Begin]
            (
                "mode_manager = "
                "aftr_mode_manager.validated_map_mode_manager_node:main"
            ),
            # [User Code End]
        ],
    },
)
