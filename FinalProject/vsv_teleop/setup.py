import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vsv_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='mrodriguez412@gatech.edu',
    description='Car-like joystick teleop for the VSV truck',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'car_teleop = vsv_teleop.car_teleop:main',
        ],
    },
)
