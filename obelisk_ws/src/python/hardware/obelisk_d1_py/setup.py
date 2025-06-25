import os
from glob import glob
from warnings import simplefilter

from setuptools import find_packages, setup, SetuptoolsDeprecationWarning

simplefilter("ignore", category=SetuptoolsDeprecationWarning)

package_name = 'obelisk_d1_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amber-lab',
    maintainer_email='ezeng@caltech.edu',
    description='Python Obelisk ROS2 package for simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "d1_example_controller = obelisk_d1_py.d1_example_controller:main"
        ],
    },
)
