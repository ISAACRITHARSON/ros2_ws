from setuptools import find_packages, setup

package_name = 'node1pkg'

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
    maintainer='isaac',
    maintainer_email='premkumar.i@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['node1 = node1pkg.node1:main','node2 = node1pkg.node2:main','node3 = node1pkg.node3:main','node4 = node1pkg.node4:main',
        ],
    },
)
