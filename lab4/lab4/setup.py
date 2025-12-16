from setuptools import setup

package_name = 'lab4'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab4_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Premkumar',
    maintainer_email='premkumar.i@northeastern.edu',
    description='Lab 4 Sensor Fusion Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_avoidance = lab4.lab4_node1_lidar_avoidance:main',
            'color_tracker = lab4.lab4_node2_color_tracker:main',
            'fusion_node = lab4.lab4_node3_fusion_node:main',
        ],
    },
)
