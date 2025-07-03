from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config', '*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sekiguchi',
    maintainer_email='ksekiguc@tcu.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "building_node = rf.rf_building.building_node:main",
            "robot_node = rf.rf_robot.robot_node:main"
        ],
    },
)
