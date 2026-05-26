import os
from glob import glob
from setuptools import find_packages, setup

package_name = "fr5_quest_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vaatsav",
    maintainer_email="awone.colab1@gmail.com",
    description="Quest2ROS controller teleoperation for the Fairino FR5 cobot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop = fr5_quest_teleop.teleop_node:main",
            "axis_check = fr5_quest_teleop.axis_check_node:main",
        ],
    },
)
