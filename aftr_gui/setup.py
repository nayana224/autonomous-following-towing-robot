# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "aftr_gui"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "gui"), glob("gui/*.ui")),
        (os.path.join("share", package_name, "images"), glob("images/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pyo",
    maintainer_email="pyo@todo.com",
    description="Operator GUI for Autonomous Following and Towing Robot mode manager services.",
    license="Apache License 2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "operator_gui = aftr_gui.refactored_operator_gui:main",
        ],
    },
)
