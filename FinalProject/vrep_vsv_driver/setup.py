from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vrep_vsv_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cedricp',
    maintainer_email='cedric.pradalier@georgiatech-metz.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_geom = vrep_vsv_driver.teleop_geom:main',
            'vsv_arm_ik = vrep_vsv_driver.vsv_arm_ik:main',
            'vsv_arm = vrep_vsv_driver.vsv_arm:main',
            'vsv_driver = vrep_vsv_driver.vsv_driver:main',
            'vsv_odom = vrep_vsv_driver.vsv_odom:main',
        ],
    },
)
