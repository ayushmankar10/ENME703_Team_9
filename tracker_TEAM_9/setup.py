import os
from glob import glob

from setuptools import find_packages, setup


package_name = "tracker_TEAM_9"

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
            ["package.xml"],
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
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Two TurtleBot 4 tracking node for Team 9.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "tracker_two_tb4 = "
            "tracker_TEAM_9.tracker_two_tb4_TEAM_9:main",
        ],
    },
)