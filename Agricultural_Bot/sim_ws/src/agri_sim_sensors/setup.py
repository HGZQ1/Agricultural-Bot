from glob import glob
from setuptools import find_packages, setup

package_name = 'agri_sim_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'NOTICE']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'normalize_mid360_cloud = agri_sim_sensors.normalize_cloud:main',
            'normalize_d405_camera_info = agri_sim_sensors.normalize_camera_info:main',
        ],
    },
)
