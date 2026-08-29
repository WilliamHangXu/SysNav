import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bev_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William Xu',
    maintainer_email='willxh.68209@gmail.com',
    description='BEV occupancy map from SLAM-registered lidar scans',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_mapper_node = bev_mapper.bev_mapper_node:main',
        ],
    },
)
