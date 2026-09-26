from setuptools import find_packages, setup

package_name = 'my_robot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iban-ubuntu',
    maintainer_email='iban-ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge = my_robot_driver.serial_bridge:main',
            'wifi_bridge = my_robot_driver.wifi_bridge:main',
            'wifi_bridge_lidar = my_robot_driver.wifi_bridge_lidar:main',
            'wifi_bridge_odom = my_robot_driver.wifi_bridge_odom:main',
            'wifi_bridge_cmd  = my_robot_driver.wifi_bridge_cmd:main',
            'fake_lidar = my_robot_driver.fake_lidar:main',
            'time_syncer = my_robot_driver.time_syncer:main',
        ],
    },
)
