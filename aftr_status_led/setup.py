# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Setup configuration for aftr_status_led."""

import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "aftr_status_led"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mechatukka",
    maintainer_email="inpyoi1304@gmail.com",
    description="GPIO-backed status LED node for Autonomous Following and Towing Robot workflow feedback.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "status_led_node = aftr_status_led.safety_status_led_node:main",
        ],
    },
)
