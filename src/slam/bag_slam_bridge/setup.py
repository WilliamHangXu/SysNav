import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bag_slam_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William Xu',
    maintainer_email='willxh.68209@gmail.com',
    description='Bridge a rosbag SLAM (LIO + TF) onto /registered_scan and /state_estimation.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_slam_bridge = bag_slam_bridge.bag_slam_bridge:main',
        ],
    },
)
