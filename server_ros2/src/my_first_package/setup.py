from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'simple_publisher = my_first_package.simple_publisher:main',
            'simple_subscriber = my_first_package.simple_subscriber:main',
            'simple_service_server = my_first_package.simple_service_server:main',
            'simple_service_client = my_first_package.simple_service_client:main',
        ],
    },
)
