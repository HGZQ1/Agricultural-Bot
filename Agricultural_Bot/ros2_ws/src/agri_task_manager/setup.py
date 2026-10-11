from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config', 'missions'),
         glob('config/missions/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'field_route_executor = '
            'agri_task_manager.field_route_executor:main',
            'generate_single_corridor_mission = '
            'agri_task_manager.generate_single_corridor_mission:main',
            'validate_field_route = '
            'agri_task_manager.validate_field_route:main',
        ],
    },
)
