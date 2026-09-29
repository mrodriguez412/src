from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vrep_ros_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cedricp',
    maintainer_email='cedric.pradalier@georgiatech-metz.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = vrep_ros_teleop.teleop:main',
            'teleop_mux_node = vrep_ros_teleop.teleop_mux:main'
        ],
    },
)
