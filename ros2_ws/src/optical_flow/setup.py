from setuptools import setup
from glob import glob

package_name = 'optical_flow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
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
            'lucas_kanade_node = optical_flow.lucas_kanade_node:main',
            'raft_node = optical_flow.raft_node:main',
            'raft_direct_node = optical_flow.raft_direct_node:main',
            'raft_small_node = optical_flow.raft_small_node:main',
            'lucas_kanade_light_node = optical_flow.lucas_kanade_light_node:main',
        ],
    },
)
