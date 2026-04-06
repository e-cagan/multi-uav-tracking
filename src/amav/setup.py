from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emin Cagan Apaydin',
    maintainer_email='cagan@todo.todo',
    description='Adaptive Multi-Agent Vision system - ROS2 nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = amav.camera_node:main',
            'detector_node = amav.detector_node:main',
            'tracker_node = amav.tracker_node:main',
            'decision_node = amav.decision_node:main',
            'coordinator_node = amav.coordinator_node:main',
        ],
    },
)
