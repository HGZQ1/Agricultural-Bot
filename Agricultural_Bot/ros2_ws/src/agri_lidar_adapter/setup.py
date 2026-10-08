from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_lidar_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agricultural Bot Team',
    maintainer_email='maintainer@example.com',
    description='MID-360 point-cloud filtering and 2-D navigation scan adapter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mid360_cloud_filter = agri_lidar_adapter.cloud_filter:main',
        ],
    },
)
