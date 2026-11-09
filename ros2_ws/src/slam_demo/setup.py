import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'slam_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install every launch file from the launch/ folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Small ROS2 demo nodes: a position publisher and subscriber',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = slam_demo.my_publisher:main',
            'my_subscriber = slam_demo.my_subscriber:main',
        ],
    },
)