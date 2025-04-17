from setuptools import setup
import os
from glob import glob

package_name = 'data_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install the resource file into the ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docker',
    maintainer_email='sivert.hb@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_handler = data_handler.data_handler_node:main',
            'realsense_accel_node = data_handler.realsense_accel_node:main',
            'inertialsense_accel_node = data_handler.inertialsense_accel_node:main',
            'calibration_node = data_handler.calibration_node:main',
            'data_recorder_node = data_handler.data_recorder_node:main',
            'depth_calc_node = data_handler.depth_calc_node:main',
        ],
    },
)
