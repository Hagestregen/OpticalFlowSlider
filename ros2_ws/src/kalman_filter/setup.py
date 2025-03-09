from setuptools import find_packages, setup

package_name = 'kalman_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'kalman_filter_node = kalman_filter.kalman_filter_node:main',
            'LFN3_kalman_filter_node = kalman_filter.LFN3_kalman_filter_node:main',
            'raft_kalman_filter_node = kalman_filter.raft_kalman_filter_node:main',
            'LK_kalman_filter_node = kalman_filter.LK_kalman_filter_node:main',
        ],
    },
)
