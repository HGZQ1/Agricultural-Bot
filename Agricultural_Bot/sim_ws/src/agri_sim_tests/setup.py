from setuptools import find_packages, setup


package_name = 'agri_sim_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agricultural Bot developers',
    maintainer_email='maintainer@example.com',
    description='Executable acceptance checks for Agricultural Bot simulation sensors.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'check_mid360 = agri_sim_tests.check_mid360:main',
            'check_d405 = agri_sim_tests.check_d405:main',
            'check_scan = agri_sim_tests.check_scan:main',
            'check_mapping = agri_sim_tests.check_mapping:main',
        ],
    },
)
