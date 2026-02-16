import os
from glob import glob

from setuptools import find_packages, setup

package_name = "task02"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/task02.launch"]),
        ("share/" + package_name + "/config", ["config/task02.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@todo.todo",
    description="Task 02 - ROS2 publisher node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sender = task02.sender:main",
        ],
    },
)
