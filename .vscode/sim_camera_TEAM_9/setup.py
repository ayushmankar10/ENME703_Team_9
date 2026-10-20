import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sim_camera_TEAM_9"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml", "README.md"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ayushman Kar",
    maintainer_email="karayushman963@gmail.com",
    description="Synthetic camera input for report pipeline testing.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "sim_camera = sim_camera_TEAM_9.sim_camera_node:main",
        ],
    },
)