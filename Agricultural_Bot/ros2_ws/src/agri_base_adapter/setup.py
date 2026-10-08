from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_base_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'velocity_gate = agri_base_adapter.velocity_gate_node:main',
            'odom_adapter = agri_base_adapter.odom_adapter_node:main',
        ],
    },
)
