import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'treasure_detector'

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
    description='Fuses metal detector readings into treasure position estimates',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'treasure_detector = treasure_detector.detector_node:main',
            'treasure_eval = treasure_detector.eval_node:main',
            'fit_sensor_model = treasure_detector.fit_sensor_model:main',
        ],
    },
)
