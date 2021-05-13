from setuptools import setup
import os
from glob import glob

package_name = 'rsb_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')), # Add launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # Add config.yaml file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmriiwa',
    maintainer_email='mortenmdahl@outlook.com',
    description='The server part of the ros2_socket_bridge package.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = rsb_server.server_node:main',
            'shutdown = rsb_server.shutdown:main',
        ],
    },
)
