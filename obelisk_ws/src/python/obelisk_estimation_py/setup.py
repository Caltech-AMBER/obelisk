import os
from glob import glob
from warnings import simplefilter

from setuptools import SetuptoolsDeprecationWarning, setup

simplefilter("ignore", category=SetuptoolsDeprecationWarning)

package_name = "obelisk_estimation_py"

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
    description="Python Obelisk ROS2 package for state estimation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "jointencoders_passthrough_estimator = obelisk_estimation_py.jointencoders_passthrough_estimator:main",
        ],
    },
)
