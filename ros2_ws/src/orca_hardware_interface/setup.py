from setuptools import setup
import os
from glob import glob

package_name = 'orca_hardware_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Swann',
    maintainer_email='your.email@example.com',
    description='ROS 2 hardware interface for ORCA Hand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orca_hardware_node = orca_hardware_interface.orca_hardware_node:main',
        ],
    },
)
