# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Setup configuration for aftr_mode_manager."""

import os
from glob import glob

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
        ("share/" + package_name, ["README.md"]),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
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
            (
                "mode_manager = "
                "aftr_mode_manager.validated_map_mode_manager_node:main"
            ),
        ],
    },
)
