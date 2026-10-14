from glob import glob
from setuptools import find_packages, setup

setup(name='agri_arm_control', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/agri_arm_control']),
                  ('share/agri_arm_control', ['package.xml', 'README.md']),
                  ('share/agri_arm_control/config', glob('config/*.yaml')),
                  ('share/agri_arm_control/launch', glob('launch/*.launch.py')),
                  ('share/agri_arm_control/reference/collaborator_2036',
                   glob('reference/collaborator_2036/*'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'harvest_arm_worker = agri_arm_control.harvest_worker:main']})
