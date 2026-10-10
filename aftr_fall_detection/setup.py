# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

# [User Code Begin]
from glob import glob
# [User Code End]

from setuptools import find_packages, setup


package_name = "aftr_fall_detection"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        # [User Code Begin]
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
        # [User Code End]
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="mechatukka",
    maintainer_email="mechatukka@example.com",
    description=(
        "RealSense RGB 기반 Autonomous Following and Towing Robot 쓰러짐 감지 패키지"
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # [User Code Begin]
            (
                "fall_detection_node = "
                "aftr_fall_detection."
                "filtered_fall_detection_node:main"
            ),
            # [User Code End]
        ],
    },
)
