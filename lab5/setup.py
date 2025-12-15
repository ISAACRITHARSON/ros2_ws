from setuptools import setup
import os
from glob import glob

package_name = 'lab5'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac',
    maintainer_email='your.email@northeastern.edu',
    description='Lab 5: Gesture-Based Robotic Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task1_turtlesim = lab5.lab5_node1_hand_control_turtlesim:main',
            'task2_tb4 = lab5.lab5_node2_hand_control_tb4_task2:main',
            'task3_obstacle = lab5.lab5_node3_hand_control_tb4_task3:main',
            'task4_navigation = lab5.lab5_node4_hand_control_tb4_task4:main',
        ],
    },
)

