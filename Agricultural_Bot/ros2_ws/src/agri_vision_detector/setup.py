from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_vision_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agricultural Bot Team',
    maintainer_email='maintainer@example.com',
    description='YOLO tomato detection and segmentation for the D405 camera.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tomato_detector = agri_vision_detector.tomato_detector:main',
        ],
    },
)
