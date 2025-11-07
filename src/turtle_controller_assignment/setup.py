from setuptools import setup

package_name = 'turtle_controller_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vlad',
    maintainer_email='r00274535@mymtu.ie',
    description='COMP9069 Assignment 1 - Turtle Control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_spawn_client = turtle_controller_assignment.turtle_spawn_client:main',
            'turtle_name_manager = turtle_controller_assignment.turtle_name_manager:main',
            'pen_control_client = turtle_controller_assignment.pen_control_client:main',
            'auto_turtle_spawner = turtle_controller_assignment.auto_turtle_spawner:main',
            'turtle_monitor_service = turtle_controller_assignment.turtle_monitor_service:main',
            'closest_turtle_service = turtle_controller_assignment.closest_turtle_service:main',
            'closest_turtle_client = turtle_controller_assignment.closest_turtle_client:main',
            'turtle_collection_server = turtle_controller_assignment.turtle_collection_server:main',
        ],
    },
)