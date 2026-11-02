from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'cefiro_hardware_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('tools',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CEFIRO Development Team',
    maintainer_email='maintainers@example.invalid',
    description='ROS 2 Jazzy Raspberry Pi USB-USART interface for the CEFIRO STM32H743 flight controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hardware_node = cefiro_hardware_interface.serial_node:main',
        ],
    },
)
