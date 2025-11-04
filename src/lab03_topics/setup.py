from setuptools import setup, find_packages
package_name = 'lab03_topics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vlad',
    maintainer_email='',
    description='Lab 03: ROS 2 topics demo',
    license='BSD',
    entry_points={
        'console_scripts': [
            'talker = lab03_topics.talker:main',
            'listener = lab03_topics.listener:main',
            'twist_relay = lab03_topics.twist_relay:main',
            'print_pose = lab03_topics.print_pose:main',
            'move_distance = lab03_topics.move_distance:main',
            'time_publisher = lab03_topics.time_publisher:main',
            'time_listener = lab03_topics.time_listener:main',
            'sphere_publisher = lab03_topics.sphere_publisher:main',
            'sphere_listener = lab03_topics.sphere_listener:main',

        ],
    },
)
