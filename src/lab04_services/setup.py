from setuptools import find_packages, setup

package_name = 'lab04_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vlad',
    maintainer_email='vladar21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'service_member_function = lab04_services.service_member_function:main',
            'service_client = lab04_services.service_client:main',
            'move_turtle_arc_server = lab04_services.move_turtle_arc_server:main',
            'move_turtle_arc_client = lab04_services.move_turtle_arc_client:main',
        ],
    },
)
