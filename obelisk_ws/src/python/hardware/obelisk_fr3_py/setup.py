import os
from glob import glob
from warnings import simplefilter

from setuptools import SetuptoolsDeprecationWarning, setup

simplefilter("ignore", category=SetuptoolsDeprecationWarning)

package_name = "obelisk_fr3_py"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alberthli",
    maintainer_email="alberthli@caltech.edu",
    description="Python Obelisk ROS2 package for controlling the FR3.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obelisk_fr3_robot = obelisk_fr3_py.obelisk_fr3_robot:main",
            "fr3_example_joint_controller = obelisk_fr3_py.fr3_example_joint_controller:main",
            # "fr3_example_ee_controller = obelisk_fr3_py.fr3_example_ee_controller:main",
        ],
    },
)
