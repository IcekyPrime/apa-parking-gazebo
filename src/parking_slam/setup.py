from setuptools import setup
import os
from glob import glob

package_name = 'parking_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zfh',
    maintainer_email='zfh@example.com',
    description='SLAM integration: TF bridge, waypoint driver, ATE evaluation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_bridge = parking_slam.tf_bridge:main',
            'demo_driver = parking_slam.demo_driver:main',
            'slam_eval = parking_slam.slam_eval:main',
        ],
    },
)
