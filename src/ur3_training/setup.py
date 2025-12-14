from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur3_training'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vlad',
    maintainer_email='vlad@example.com',
    description='Package for UR3 robot manipulation training',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_position_control = ur3_training.joint_position_control:main',
            'joint_trajectory_control = ur3_training.joint_trajectory_control:main',
        ],
    },
)