import os
from glob import glob
from warnings import simplefilter

from setuptools import SetuptoolsDeprecationWarning, setup

simplefilter("ignore", category=SetuptoolsDeprecationWarning)

package_name = "obelisk_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        # this is how you expose data directories in your package
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yml"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.obk"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alberthli",
    maintainer_email="alberthli@caltech.edu",
    description="Main ROS2 Obelisk package for launching the whole stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_state = obelisk_ros.global_state:main",
        ],
    },
)
