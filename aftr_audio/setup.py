# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Setup configuration for aftr_audio."""


# [User Code Begin]
from glob import glob
import os
# [User Code End]

from setuptools import find_packages
from setuptools import setup


package_name = "aftr_audio"


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
        # [User Code Begin]
        ("share/" + package_name, ["README.md"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "sounds"),
            glob(os.path.join("sounds", "*.wav")),
        ),
        # [User Code End]
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pyo",
    maintainer_email="inpyoi1304@gmail.com",
    description="Audio feedback package for Autonomous Following and Towing Robot autonomous driving events.",
    license="Apache License 2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            # [User Code Begin]
            "audio_node = aftr_audio.audio_node:main",
            # [User Code End]
        ],
    },
)
