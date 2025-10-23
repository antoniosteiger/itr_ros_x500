import os
from glob import glob
from setuptools import setup

package_name = 'itr_mission_x500'

setup(
    name=package_name,
    version='0.0.1',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Antonio Steiger',
    author_email='antonio.steiger@tum.de',
    description='Main Mission Operations Code for x500 Drone at ITR. Launches State Machine, Controller, Logging etc.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
        ],
    },
)