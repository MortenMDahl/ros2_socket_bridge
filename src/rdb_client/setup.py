from setuptools import setup

package_name = 'rdb_client'

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
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = rdb_client.client_node:main'
        ],
    },
)
asdf 
