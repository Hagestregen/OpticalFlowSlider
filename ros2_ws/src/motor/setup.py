from setuptools import setup
import os
from glob import glob

package_name = 'motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/motor_control.py']),
        ('lib/' + package_name, [package_name+'/utils.py']),
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
            'motor_node = motor.motor_node:main',
            'motor_publisher_node = motor.motor_publisher_node:main',
            'mock_motor_node = motor.mock_motor_node:main',
        ],
    },
)
